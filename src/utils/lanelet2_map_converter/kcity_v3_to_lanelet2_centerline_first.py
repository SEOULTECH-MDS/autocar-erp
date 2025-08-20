import argparse
import json
import math
import os
from collections import defaultdict
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import geopandas as gpd
import numpy as np
from lxml import etree
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import split, linemerge
from shapely.strtree import STRtree
import shapely

try:
    import mgrs  # For MGRS grid generation
except Exception:  # pragma: no cover
    mgrs = None


# -------------------- Configuration (override via CLI) --------------------

DEFAULT_LANE_WIDTH_M = 3.5
DEFAULT_LINE_WIDTH_TAG_M = 0.200
DEFAULT_LOCATION = "urban"
DEFAULT_ONE_WAY = "yes"
DEFAULT_SPEED_LIMIT = 30

MIN_SEGMENT_LENGTH_M = 6.0
MAX_SEGMENT_LENGTH_M = 80.0
MERGE_DIR_THRESHOLD_DEG = 20.0
GAP_BRIDGE_MAX_M = 2.0

BOUNDARY_SAMPLE_STEP_M = 5.0
BOUNDARY_SEARCH_BUFFER_M = 4.0
# Boundary candidate filters
BOUNDARY_PARALLEL_THRESH_DEG = 25.0
BOUNDARY_MIN_OFFSET_M = 0.2
BOUNDARY_MAX_OFFSET_M_FACTOR = 1.2  # relative to lane_width
SMALL_GAP_SNAP_M = 1.0

SUCCESSOR_SEARCH_RADIUS_M = 3.0
SUCCESSOR_DIR_THRESHOLD_DEG = 45.0
LATERAL_OFFSET_TOL_RATIO = 0.7  # of lane width
SUCCESSOR_VEC_ALIGN_DEG = 30.0
INTERSECTION_SNAP_RADIUS_M = 2.0
INTERSECTION_DIR_THRESHOLD_DEG = 120.0

COORD_ROUND_DECIMALS = 7


# --------------------------- Helper data models ---------------------------


@dataclass
class Way:
    node_refs: List[str]
    tags: Dict[str, str]


@dataclass
class Relation:
    members: List[Tuple[str, str, str]]  # (type, ref, role)
    tags: Dict[str, str]


class IdGenerator:
    def __init__(self) -> None:
        self.node_id = 1
        self.way_id = 1
        self.rel_id = 1

    def next_node(self) -> str:
        nid = str(self.node_id)
        self.node_id += 1
        return nid

    def next_way(self) -> str:
        wid = str(self.way_id)
        self.way_id += 1
        return wid

    def next_relation(self) -> str:
        rid = str(self.rel_id)
        self.rel_id += 1
        return rid


class OsmBuilder:
    def __init__(self) -> None:
        self.nodes: Dict[str, Tuple[float, float]] = {}
        self.node_coord_to_id: Dict[Tuple[float, float], str] = {}
        self.ways: Dict[str, Way] = {}
        self.relations: Dict[str, Relation] = {}
        self.ids = IdGenerator()

    def _round_coord(self, lon: float, lat: float) -> Tuple[float, float]:
        return (round(lon, COORD_ROUND_DECIMALS), round(lat, COORD_ROUND_DECIMALS))

    def add_node(self, lon: float, lat: float) -> str:
        coord = self._round_coord(lon, lat)
        if coord in self.node_coord_to_id:
            return self.node_coord_to_id[coord]
        nid = self.ids.next_node()
        self.nodes[nid] = coord
        self.node_coord_to_id[coord] = nid
        return nid

    def add_way(self, coords: List[Tuple[float, float]], tags: Optional[Dict[str, str]] = None) -> Optional[str]:
        if not coords:
            return None
        # Deduplicate consecutive coordinates
        dedup_coords: List[Tuple[float, float]] = [self._round_coord(*coords[0])]
        for lon, lat in coords[1:]:
            r = self._round_coord(lon, lat)
            if r != dedup_coords[-1]:
                dedup_coords.append(r)
        if len(dedup_coords) < 2:
            # Allow single-node ways for specific marker types (to mimic sample map)
            t = (tags or {}).get("type") if tags else None
            if t not in {"light_bulbs", "traffic_light"}:
                return None
        node_refs = [self.add_node(lon, lat) for lon, lat in dedup_coords]
        wid = self.ids.next_way()
        self.ways[wid] = Way(node_refs=node_refs, tags=tags or {})
        return wid

    def add_relation(self, members: List[Tuple[str, str, str]], tags: Optional[Dict[str, str]] = None) -> str:
        rid = self.ids.next_relation()
        self.relations[rid] = Relation(members=members, tags=tags or {})
        return rid

    def save_osm(self, output_path: str) -> None:
        osm_root = etree.Element("osm", version="0.6", generator="KCityV3ToLanelet2")
        for node_id, (lon, lat) in self.nodes.items():
            etree.SubElement(osm_root, "node", id=node_id, lat=str(lat), lon=str(lon))
        for way_id, way in self.ways.items():
            way_elem = etree.SubElement(osm_root, "way", id=way_id)
            for nref in way.node_refs:
                etree.SubElement(way_elem, "nd", ref=nref)
            for k, v in way.tags.items():
                etree.SubElement(way_elem, "tag", k=str(k), v=str(v))
        for rel_id, rel in self.relations.items():
            rel_elem = etree.SubElement(osm_root, "relation", id=rel_id)
            for m_type, m_ref, m_role in rel.members:
                etree.SubElement(rel_elem, "member", type=m_type, ref=m_ref, role=m_role)
            for k, v in rel.tags.items():
                etree.SubElement(rel_elem, "tag", k=str(k), v=str(v))
        xml_str = etree.tostring(osm_root, pretty_print=True, xml_declaration=True, encoding="UTF-8")
        with open(output_path, "wb") as f:
            f.write(xml_str)


# ----------------------------- Geometry utils -----------------------------


def _line_length_m(line: LineString) -> float:
    # Approximate meters using WGS84: 1 deg lat ~ 111,111 m; lon scaled by cos(lat)
    if line.is_empty:
        return 0.0
    length = 0.0
    coords = [(c[0], c[1]) for c in line.coords]
    for (x1, y1), (x2, y2) in zip(coords[:-1], coords[1:]):
        dx = (x2 - x1) * math.cos(math.radians((y1 + y2) / 2.0))
        dy = (y2 - y1)
        length += math.hypot(dx, dy)
    return length * 111111.0


def _sample_line_by_distance(line: LineString, step_m: float) -> List[Tuple[float, float]]:
    total = _line_length_m(line)
    if total == 0.0:
        return [(c[0], c[1]) for c in line.coords]
    # Convert meters to fraction of line length
    coords: List[Tuple[float, float]] = []
    # Work in param with shapely to interpolate by relative distance [0,1]
    num = max(2, int(math.ceil(total / step_m)) + 1)
    for i in range(num):
        d = min(1.0, i / (num - 1))
        p = line.interpolate(d, normalized=True)
        coords.append((p.x, p.y))
    return coords


def _heading_deg(line: LineString) -> float:
    coords = [(c[0], c[1]) for c in line.coords]
    if len(coords) < 2:
        return 0.0
    (x1, y1), (x2, y2) = coords[0], coords[-1]
    dx = (x2 - x1) * math.cos(math.radians((y1 + y2) / 2.0))
    dy = (y2 - y1)
    ang = math.degrees(math.atan2(dy, dx))
    return (ang + 360.0) % 360.0


def _resample_line_equal_step(line: LineString, step_m: float) -> List[Tuple[float, float]]:
    """Return points sampled approximately every step_m along the line, including endpoints."""
    total = _line_length_m(line)
    if total <= 0.0:
        return [(c[0], c[1]) for c in line.coords]
    num = max(2, int(math.ceil(total / step_m)) + 1)
    pts: List[Tuple[float, float]] = []
    for i in range(num):
        d = min(1.0, i / (num - 1))
        p = line.interpolate(d, normalized=True)
        pts.append((p.x, p.y))
    return pts


def _chaikin_smooth(points: List[Tuple[float, float]], iterations: int = 2, keep_endpoints: bool = True) -> List[Tuple[float, float]]:
    if len(points) < 3 or iterations <= 0:
        return points
    pts = points[:]
    for _ in range(iterations):
        new_pts: List[Tuple[float, float]] = []
        if keep_endpoints:
            new_pts.append(pts[0])
        for i in range(len(pts) - 1):
            x0, y0 = pts[i]
            x1, y1 = pts[i + 1]
            qx, qy = 0.75 * x0 + 0.25 * x1, 0.75 * y0 + 0.25 * y1
            rx, ry = 0.25 * x0 + 0.75 * x1, 0.25 * y0 + 0.75 * y1
            new_pts.append((qx, qy))
            new_pts.append((rx, ry))
        if keep_endpoints:
            new_pts.append(pts[-1])
        pts = new_pts
    return pts


def _angle_diff_deg(a: float, b: float) -> float:
    d = abs((a - b + 180.0) % 360.0 - 180.0)
    return d


def _offset_point(p: Point, heading_deg: float, offset_m: float) -> Point:
    # left positive offset when heading increases CCW (north-up lat/lon)
    rad = math.radians(heading_deg + 90.0)
    # meters to degrees
    m_per_deg_lat = 111111.0
    m_per_deg_lon = 111111.0 * math.cos(math.radians(p.y))
    dx = (offset_m * math.cos(rad)) / m_per_deg_lon
    dy = (offset_m * math.sin(rad)) / m_per_deg_lat
    return Point(p.x + dx, p.y + dy)


def _nearest_line(point: Point, tree: STRtree, candidates: List[LineString], max_dist_m: float) -> Optional[LineString]:
    if not candidates:
        return None
    # Query bbox first
    buf = _point_buffer_deg(point, max_dist_m)
    hits = tree.query(buf)
    best = None
    best_d = float("inf")
    for item in hits:
        # Shapely STRtree may return either geometries or indices depending on version
        geom = item
        try:
            import numpy as _np  # local scope
            if isinstance(item, (int, _np.integer)):
                geom = tree.geometries[item]
        except Exception:
            pass
        d_m = _geodesic_point_line_distance_m(point, geom)
        if d_m < best_d and d_m <= max_dist_m:
            best = geom
            best_d = d_m
    return best


def _geodesic_point_line_distance_m(point: Point, line: LineString) -> float:
    # Approx geodesic via degree scaling to meters
    p_proj = line.interpolate(line.project(point))
    dx = (p_proj.x - point.x) * math.cos(math.radians((p_proj.y + point.y) / 2.0))
    dy = (p_proj.y - point.y)
    return math.hypot(dx, dy) * 111111.0


def _point_buffer_deg(point: Point, radius_m: float) -> Polygon:
    # Create an approximate buffer in degrees by converting meters to deg at point latitude
    m_per_deg_lat = 111111.0
    m_per_deg_lon = 111111.0 * math.cos(math.radians(point.y))
    dx = radius_m / m_per_deg_lon
    dy = radius_m / m_per_deg_lat
    return shapely.geometry.box(point.x - dx, point.y - dy, point.x + dx, point.y + dy)


def _coords_xy(geom: LineString) -> List[Tuple[float, float]]:
    return [(c[0], c[1]) for c in geom.coords]


def _geodesic_point_point_distance_m(p1: Point, p2: Point) -> float:
    dx = (p2.x - p1.x) * math.cos(math.radians((p1.y + p2.y) / 2.0))
    dy = (p2.y - p1.y)
    return math.hypot(dx, dy) * 111111.0


def _angle_between_heading_and_vector_deg(heading_deg: float, origin: Point, target: Point) -> float:
    # Vector from origin to target in meters-scaled coordinates
    dx = (target.x - origin.x) * math.cos(math.radians((origin.y + target.y) / 2.0))
    dy = (target.y - origin.y)
    v = np.array([dx, dy])
    if np.linalg.norm(v) < 1e-9:
        return 0.0
    v = v / np.linalg.norm(v)
    h = np.array([
        math.cos(math.radians(heading_deg)),
        math.sin(math.radians(heading_deg)),
    ])
    h = h / np.linalg.norm(h)
    dot = float(np.clip(np.dot(v, h), -1.0, 1.0))
    return math.degrees(math.acos(dot))


def _signed_lateral_offset_m(ref_heading_deg: float, origin: Point, target: Point) -> float:
    # +: target is to the left of the heading from origin
    dx = (target.x - origin.x) * math.cos(math.radians((origin.y + target.y) / 2.0))
    dy = (target.y - origin.y)
    vec = np.array([dx, dy])
    fwd = np.array([math.cos(math.radians(ref_heading_deg)), math.sin(math.radians(ref_heading_deg))])
    # Perp-left unit vector
    left = np.array([-fwd[1], fwd[0]])
    return float(np.dot(vec, left))


def _heading_between_points_deg(a: Point, b: Point) -> float:
    dx = (b.x - a.x) * math.cos(math.radians((a.y + b.y) / 2.0))
    dy = (b.y - a.y)
    ang = math.degrees(math.atan2(dy, dx))
    return (ang + 360.0) % 360.0


def _interpolate_from_start_m(line: LineString, dist_m: float) -> Point:
    total = _line_length_m(line)
    if total <= 0.0:
        return Point(line.coords[0])
    t = max(0.0, min(1.0, dist_m / total))
    p = line.interpolate(t, normalized=True)
    return Point(p.x, p.y)


def _interpolate_from_end_m(line: LineString, dist_m: float) -> Point:
    total = _line_length_m(line)
    if total <= 0.0:
        return Point(line.coords[-1])
    t = max(0.0, min(1.0, 1.0 - (dist_m / total)))
    p = line.interpolate(t, normalized=True)
    return Point(p.x, p.y)


def _circle_from_three_points(p1: Point, p2: Point, p3: Point) -> Optional[Tuple[Point, float]]:
    # Compute circle center and radius through 3 non-colinear points
    x1, y1 = p1.x, p1.y
    x2, y2 = p2.x, p2.y
    x3, y3 = p3.x, p3.y
    a = x1 * (y2 - y3) - y1 * (x2 - x3) + x2 * y3 - x3 * y2
    if abs(a) < 1e-12:
        return None
    b = (x1 ** 2 + y1 ** 2)
    c = (x2 ** 2 + y2 ** 2)
    d = (x3 ** 2 + y3 ** 2)
    ux = (b * (y2 - y3) + c * (y3 - y1) + d * (y1 - y2)) / (2 * a)
    uy = (b * (x3 - x2) + c * (x1 - x3) + d * (x2 - x1)) / (2 * a)
    center = Point(ux, uy)
    r = _geodesic_point_point_distance_m(center, p1)
    return center, r


def _arc_points(center: Point, start: Point, end: Point, clockwise: bool, step_deg: float = 10.0) -> List[Tuple[float, float]]:
    # produce points from start to end along arc around center
    def angle(p: Point) -> float:
        return math.atan2(p.y - center.y, (p.x - center.x))
    ang_s = angle(start)
    ang_e = angle(end)
    # normalize
    if clockwise:
        if ang_e > ang_s:
            ang_e -= 2 * math.pi
        angs = np.arange(ang_s, ang_e - 1e-9, -math.radians(step_deg))
    else:
        if ang_e < ang_s:
            ang_e += 2 * math.pi
        angs = np.arange(ang_s, ang_e + 1e-9, math.radians(step_deg))
    pts: List[Tuple[float, float]] = []
    for a in angs:
        x = center.x + math.cos(a) * (start.distance(center))
        y = center.y + math.sin(a) * (start.distance(center))
        pts.append((x, y))
    pts.append((end.x, end.y))
    return pts


# --------------------------- Core conversion logic ---------------------------


class KCityV3Converter:
    def __init__(
        self,
        input_dir: str,
        output_osm: str,
        output_projector_yaml: Optional[str] = None,
        output_map_config_yaml: Optional[str] = None,
        lane_width_m: float = DEFAULT_LANE_WIDTH_M,
        line_width_tag_m: float = DEFAULT_LINE_WIDTH_TAG_M,
        refine_sequences: Optional[List[List[str]]] = None,
    ) -> None:
        self.input_dir = input_dir
        self.output_osm = output_osm
        self.output_projector_yaml = output_projector_yaml
        self.output_map_config_yaml = output_map_config_yaml
        self.lane_width_m = lane_width_m
        self.line_width_tag_m = line_width_tag_m
        self.refine_sequences: List[List[str]] = refine_sequences or []

        self.osm = OsmBuilder()

        self.gdf: Dict[str, gpd.GeoDataFrame] = {}
        self.centerline_segments: List[LineString] = []
        self.lanelet_relations: List[str] = []  # relation ids
        self.lanelet_centers: Dict[str, LineString] = {}
        self.rel_left_right_geom: Dict[str, Tuple[LineString, LineString]] = {}
        self.endpoint_clusters: List[Point] = []
        self.manual_successors: Dict[str, List[str]] = {}
        self.manual_predecessors: Dict[str, List[str]] = {}

    # --------------------- Load & preprocess ---------------------
    def load_layers(self) -> None:
        def _load(name: str) -> Optional[gpd.GeoDataFrame]:
            path = os.path.join(self.input_dir, f"{name}.shp")
            if not os.path.exists(path):
                return None
            gdf = gpd.read_file(path)
            if gdf.crs is not None and gdf.crs.to_epsg() != 4326:
                gdf = gdf.to_crs("EPSG:4326")
            return gdf

        for name in [
            "link",
            "left_right_lines",
            "dased_lines",
            "dashed_lines",  # fallback name if typo fixed
            "stopline",
            "road_marking_lines",
            "sidewalk_area",
            "traffic_location",
        ]:
            gdf = _load(name)
            if gdf is not None:
                # keep only valid geometries
                gdf = gdf[~gdf.geometry.is_empty & gdf.geometry.is_valid]
                self.gdf[name] = gdf

    # --------------------- Topology & segmentation ---------------------
    def segment_links(self) -> None:
        link_gdf = self.gdf.get("link")
        if link_gdf is None or link_gdf.empty:
            return

        # Merge multi-part to single LineStrings, then split at intersections, stoplines, crosswalks, and maximum length
        lines: List[LineString] = []
        for geom in link_gdf.geometry:
            if geom is None or geom.is_empty:
                continue
            if geom.type in ("MultiLineString",):
                merged = linemerge(geom)
                if isinstance(merged, LineString):
                    lines.append(merged)
                else:
                    lines.extend(list(merged.geoms))
            elif isinstance(geom, LineString):
                lines.append(geom)

        # Split at intersections between link lines
        merged_all = linemerge(lines)
        if isinstance(merged_all, LineString):
            work_lines = [merged_all]
        else:
            work_lines = list(merged_all.geoms)

        # Prepare splitters: stoplines and crosswalk boundaries
        splitters: List[LineString] = []
        if "stopline" in self.gdf:
            splitters.extend([g for g in self.gdf["stopline"].geometry if isinstance(g, LineString)])
        # NOTE: Do NOT split by crosswalk/sidewalk polygons to keep roads continuous across crosswalks

        segmented: List[LineString] = []
        for base in work_lines:
            cur = base
            for sl in splitters:
                try:
                    res = split(cur, sl)
                    # split returns GeometryCollection; continue splitting each part with remaining splitters
                    if len(res.geoms) > 1:
                        cur_parts = list(res.geoms)
                        # recursively split all parts by remaining splitters
                        for more in splitters:
                            next_parts: List[LineString] = []
                            for p in cur_parts:
                                try:
                                    r = split(p, more)
                                    next_parts.extend(list(r.geoms))
                                except Exception:
                                    next_parts.append(p)
                            cur_parts = next_parts
                        for p in cur_parts:
                            if isinstance(p, LineString) and _line_length_m(p) > 0.0:
                                segmented.append(p)
                        cur = None
                        break
                except Exception:
                    continue
            if isinstance(cur, LineString) and _line_length_m(cur) > 0.0:
                segmented.append(cur)

        # Enforce max segment length by cutting at regular intervals
        final_segments: List[LineString] = []
        for seg in segmented:
            length_m = _line_length_m(seg)
            if length_m <= MAX_SEGMENT_LENGTH_M:
                final_segments.append(seg)
                continue
            # Cut into chunks by fraction
            num = int(math.ceil(length_m / MAX_SEGMENT_LENGTH_M))
            for i in range(num):
                a = i / num
                b = min(1.0, (i + 1) / num)
                part = shapely.ops.substring(seg, a, b, normalized=True)
                if isinstance(part, LineString) and _line_length_m(part) > 0.5:
                    final_segments.append(part)

        # Filter too-short and pathological segments
        keep: List[LineString] = []
        for seg in final_segments:
            if _line_length_m(seg) < MIN_SEGMENT_LENGTH_M:
                continue
            if seg.is_ring or seg.crosses(seg):
                # self-loop or self-crossing: skip
                continue
            keep.append(seg)

        # Merge short sequential segments and bridge small gaps
        merged = self._merge_and_bridge_segments(keep)
        # Snap endpoints in intersections to common cluster centroids for continuity
        self.endpoint_clusters = self._build_endpoint_clusters(merged)
        self.centerline_segments = self._snap_endpoints_to_clusters(merged, self.endpoint_clusters)

    def _endpoints_and_heading(self, seg: LineString) -> Tuple[Point, Point, float]:
        s = Point(seg.coords[0])
        e = Point(seg.coords[-1])
        return s, e, _heading_deg(seg)

    def _merge_and_bridge_segments(self, segments: List[LineString]) -> List[LineString]:
        segs = segments[:]
        changed = True
        while changed:
            changed = False
            used = [False] * len(segs)
            new_list: List[LineString] = []
            for i, a in enumerate(segs):
                if used[i]:
                    continue
                s_a, e_a, h_a = self._endpoints_and_heading(a)
                best_j = -1
                best_score = float("inf")
                # find forward neighbor whose start is near our end
                for j, b in enumerate(segs):
                    if i == j or used[j]:
                        continue
                    s_b, e_b, h_b = self._endpoints_and_heading(b)
                    d = _geodesic_point_point_distance_m(e_a, s_b)
                    if d > GAP_BRIDGE_MAX_M:
                        continue
                    if _angle_diff_deg(h_a, h_b) > MERGE_DIR_THRESHOLD_DEG:
                        continue
                    # prefer closest
                    if d < best_score:
                        best_score = d
                        best_j = j
                if best_j >= 0:
                    # bridge if needed, then concatenate
                    b = segs[best_j]
                    s_b, _, _ = self._endpoints_and_heading(b)
                    bridge = None
                    if best_score > 0.05:
                        bridge = LineString([(e_a.x, e_a.y), (s_b.x, s_b.y)])
                    coords = list(a.coords)
                    if bridge is not None:
                        coords += list(bridge.coords)
                    coords += list(b.coords)
                    merged = LineString(coords)
                    if _line_length_m(merged) > 0.5:
                        new_list.append(merged)
                        used[i] = True
                        used[best_j] = True
                        changed = True
                        continue
                # no merge partner
                new_list.append(a)
                used[i] = True
            segs = new_list
        return segs

    def _build_endpoint_clusters(self, segments: List[LineString]) -> List[Point]:
        pts: List[Point] = []
        for seg in segments:
            pts.append(Point(seg.coords[0]))
            pts.append(Point(seg.coords[-1]))
        clusters: List[List[Point]] = []
        for p in pts:
            placed = False
            for cluster in clusters:
                # compare with cluster representative (centroid) distance
                cx = sum(pt.x for pt in cluster) / len(cluster)
                cy = sum(pt.y for pt in cluster) / len(cluster)
                center = Point(cx, cy)
                if _geodesic_point_point_distance_m(p, center) <= INTERSECTION_SNAP_RADIUS_M:
                    cluster.append(p)
                    placed = True
                    break
            if not placed:
                clusters.append([p])
        # compute centroids
        centers: List[Point] = []
        for cluster in clusters:
            if len(cluster) < 2:
                continue
            cx = sum(pt.x for pt in cluster) / len(cluster)
            cy = sum(pt.y for pt in cluster) / len(cluster)
            centers.append(Point(cx, cy))
        return centers

    def _snap_endpoints_to_clusters(self, segments: List[LineString], centers: List[Point]) -> List[LineString]:
        if not centers:
            return segments
        snapped: List[LineString] = []
        for seg in segments:
            coords = [(c[0], c[1]) for c in seg.coords]
            # start
            p0 = Point(coords[0][0], coords[0][1])
            best0 = None
            bestd0 = INTERSECTION_SNAP_RADIUS_M + 1.0
            for c in centers:
                d = _geodesic_point_point_distance_m(p0, c)
                if d < bestd0 and d <= INTERSECTION_SNAP_RADIUS_M:
                    best0 = c
                    bestd0 = d
            if best0 is not None:
                coords[0] = (best0.x, best0.y)
            # end
            p1 = Point(coords[-1][0], coords[-1][1])
            best1 = None
            bestd1 = INTERSECTION_SNAP_RADIUS_M + 1.0
            for c in centers:
                d = _geodesic_point_point_distance_m(p1, c)
                if d < bestd1 and d <= INTERSECTION_SNAP_RADIUS_M:
                    best1 = c
                    bestd1 = d
            if best1 is not None:
                coords[-1] = (best1.x, best1.y)
            snapped.append(LineString(coords))
        return snapped

    # --------------------- Boundary matching ---------------------
    def _prepare_boundary_trees(self) -> Tuple[List[LineString], Optional[STRtree], List[LineString], Optional[STRtree]]:
        solid_lines: List[LineString] = []
        dashed_lines: List[LineString] = []
        if "left_right_lines" in self.gdf:
            solid_lines = [g for g in self.gdf["left_right_lines"].geometry if isinstance(g, LineString)]
        dashed_name = "dashed_lines" if "dashed_lines" in self.gdf else "dased_lines"
        if dashed_name in self.gdf:
            dashed_lines = [g for g in self.gdf[dashed_name].geometry if isinstance(g, LineString)]
        solid_tree = STRtree(solid_lines) if solid_lines else None
        dashed_tree = STRtree(dashed_lines) if dashed_lines else None
        return solid_lines, solid_tree, dashed_lines, dashed_tree

    def _make_offset_boundary(self, seg: LineString, side: str) -> LineString:
        heading = _heading_deg(seg)
        sign = 1.0 if side == "left" else -1.0
        coords = _sample_line_by_distance(seg, BOUNDARY_SAMPLE_STEP_M)
        pts = []
        for x, y in coords:
            p = Point(x, y)
            pts.append(_offset_point(p, heading, sign * self.lane_width_m / 2.0))
        return LineString([(p.x, p.y) for p in pts])

    def _match_boundary(self, seg: LineString, side: str, solid_tree: Optional[STRtree], solid_lines: List[LineString], dashed_tree: Optional[STRtree], dashed_lines: List[LineString]) -> Tuple[LineString, Dict[str, str]]:
        # Sample along segment and query nearest boundary candidates
        samples = _sample_line_by_distance(seg, BOUNDARY_SAMPLE_STEP_M)
        heading = _heading_deg(seg)
        lat_off_tol_m = self.lane_width_m * LATERAL_OFFSET_TOL_RATIO
        votes: Dict[int, int] = defaultdict(int)  # index in candidate list

        def collect_votes(tree: Optional[STRtree], cands: List[LineString], is_solid: bool) -> Optional[LineString]:
            if tree is None or not cands:
                return None
            cand_scores: List[Tuple[float, int]] = []  # (score, index)
            for j, cand in enumerate(cands):
                # filter by parallelism
                if _angle_diff_deg(_heading_deg(seg), _heading_deg(cand)) > BOUNDARY_PARALLEL_THRESH_DEG:
                    continue
                off_list: List[float] = []
                side_ok_votes = 0
                start_ok = False
                end_ok = False
                for x, y in samples:
                    p = Point(x, y)
                    proj = cand.interpolate(cand.project(p))
                    # side check
                    vec_seg = np.array([
                        math.cos(math.radians(heading)),
                        math.sin(math.radians(heading)),
                    ])
                    vec_side = np.array([
                        (proj.x - p.x) * math.cos(math.radians((proj.y + p.y) / 2.0)),
                        (proj.y - p.y),
                    ])
                    cross = np.cross(vec_seg, vec_side)
                    if side == "left" and cross <= 0:
                        continue
                    if side == "right" and cross >= 0:
                        continue
                    off = _geodesic_point_point_distance_m(p, proj)
                    # offset gate: near-expected corridor
                    if off < BOUNDARY_MIN_OFFSET_M or off > max(lat_off_tol_m, self.lane_width_m * BOUNDARY_MAX_OFFSET_M_FACTOR):
                        continue
                    off_list.append(off)
                    side_ok_votes += 1
                # enforce start/end anchoring
                p_start = Point(samples[0][0], samples[0][1])
                proj_start = cand.interpolate(cand.project(p_start))
                off_start = _geodesic_point_point_distance_m(p_start, proj_start)
                start_ok = off_start <= lat_off_tol_m
                p_end = Point(samples[-1][0], samples[-1][1])
                proj_end = cand.interpolate(cand.project(p_end))
                off_end = _geodesic_point_point_distance_m(p_end, proj_end)
                end_ok = off_end <= lat_off_tol_m
                # require candidate to follow along most of the segment
                if not (start_ok and end_ok):
                    continue
                if side_ok_votes < max(3, int(len(samples) * 0.6)):
                    continue
                # score: lower median offset preferred, slight penalty on angle diff
                median_off = float(np.median(off_list)) if off_list else 1e9
                angle_penalty = _angle_diff_deg(_heading_deg(seg), _heading_deg(cand)) * 0.01
                score = median_off + angle_penalty
                cand_scores.append((score, j))
            if not cand_scores:
                return None
            cand_scores.sort(key=lambda x: x[0])
            return cands[cand_scores[0][1]]

        # Try solid lines first
        best = collect_votes(solid_tree, solid_lines, True)
        subtype = "solid" if best is not None else "dashed"
        if best is None:
            best = collect_votes(dashed_tree, dashed_lines, False)
        if best is None:
            # fallback to offset boundary
            bline = self._make_offset_boundary(seg, side)
            tags = {"type": "line_thin", "subtype": "solid", "width": f"{self.line_width_tag_m:.3f}"}
            return bline, tags
        else:
            # Compose boundary by following candidate but filling holes with offset points
            composed_pts: List[Tuple[float, float]] = []
            lat_off_tol_m = self.lane_width_m * LATERAL_OFFSET_TOL_RATIO * 1.2
            heading = _heading_deg(seg)
            for x, y in samples:
                p = Point(x, y)
                proj = best.interpolate(best.project(p))
                off = _geodesic_point_line_distance_m(p, best)
                if off <= lat_off_tol_m:
                    composed_pts.append((proj.x, proj.y))
                else:
                    # use offset fallback at this position
                    q = _offset_point(p, heading, (1.0 if side == "left" else -1.0) * self.lane_width_m / 2.0)
                    composed_pts.append((q.x, q.y))
            bline = LineString(composed_pts)
            tags = {"type": "line_thin", "subtype": subtype, "width": f"{self.line_width_tag_m:.3f}"}
            if subtype == "dashed":
                tags["lane_change"] = "yes"
            return bline, tags

    # --------------------- Build lanelets ---------------------
    def build_lanelets(self) -> None:
        if not self.centerline_segments:
            return
        solid_lines, solid_tree, dashed_lines, dashed_tree = self._prepare_boundary_trees()

        for seg in self.centerline_segments:
            left_geom, left_tags = self._match_boundary(seg, "left", solid_tree, solid_lines, dashed_tree, dashed_lines)
            right_geom, right_tags = self._match_boundary(seg, "right", solid_tree, solid_lines, dashed_tree, dashed_lines)

            # Build ways in OSM using raw geometries to preserve intended shapes
            left_way_id = self.osm.add_way(_coords_xy(left_geom), left_tags)
            right_way_id = self.osm.add_way(_coords_xy(right_geom), right_tags)
            if not left_way_id or not right_way_id:
                continue

            # Relation (lanelet)
            tags = {
                "type": "lanelet",
                "subtype": "road",
                "one_way": DEFAULT_ONE_WAY,
                "speed_limit": str(DEFAULT_SPEED_LIMIT),
                "location": DEFAULT_LOCATION,
            }
            rel_id = self.osm.add_relation(
                members=[
                    ("way", left_way_id, "left"),
                    ("way", right_way_id, "right"),
                ],
                tags=tags,
            )
            self.lanelet_relations.append(rel_id)
            self.lanelet_centers[rel_id] = seg
            self.rel_left_right_geom[rel_id] = (left_geom, right_geom)

    def _rebuild_way_geometry(self, way_id: str) -> LineString:
        way = self.osm.ways[way_id]
        coords = [(self.osm.nodes[nid][0], self.osm.nodes[nid][1]) for nid in way.node_refs]
        return LineString(coords)

    def _set_relation_members(self, rel_id: str, left_way_id: str, right_way_id: str) -> None:
        self.osm.relations[rel_id].members = [("way", left_way_id, "left"), ("way", right_way_id, "right")]

    def fix_lanelet_orientation_and_width(self) -> None:
        w_min = 2.5
        w_max = 5.0
        for rid in list(self.lanelet_relations):
            center = self.lanelet_centers.get(rid)
            if center is None:
                continue
            rel = self.osm.relations.get(rid)
            if rel is None:
                continue
            # Extract member way ids
            left_way_id = None
            right_way_id = None
            for t, ref, role in rel.members:
                if role == "left":
                    left_way_id = ref
                elif role == "right":
                    right_way_id = ref
            if not left_way_id or not right_way_id:
                continue
            left_geom = self._rebuild_way_geometry(left_way_id)
            right_geom = self._rebuild_way_geometry(right_way_id)
            # Sample along center and measure signed offsets
            samples = _sample_line_by_distance(center, max(BOUNDARY_SAMPLE_STEP_M, 2.0))
            hdg = _heading_deg(center)
            left_signs = []
            right_signs = []
            widths = []
            for x, y in samples:
                p = Point(x, y)
                lp = left_geom.interpolate(left_geom.project(p))
                rp = right_geom.interpolate(right_geom.project(p))
                left_signs.append(_signed_lateral_offset_m(hdg, p, lp))
                right_signs.append(_signed_lateral_offset_m(hdg, p, rp))
                widths.append(_geodesic_point_point_distance_m(lp, rp))
            mean_left = float(np.mean(left_signs))
            mean_right = float(np.mean(right_signs))
            median_width = float(np.median(widths)) if widths else self.lane_width_m
            swapped = False
            if mean_left < 0 or mean_right > 0:
                # swap roles
                self._set_relation_members(rid, right_way_id, left_way_id)
                swapped = True
            # width sanity: if out of range, rebuild from offsets
            if median_width < w_min or median_width > w_max:
                # rebuild both sides from offsets
                new_left = self._make_offset_boundary(center, "left")
                new_right = self._make_offset_boundary(center, "right")
                new_left_id = self.osm.add_way([(c[0], c[1]) for c in new_left.coords], {"type": "line_thin", "subtype": "solid", "width": f"{self.line_width_tag_m:.3f}"})
                new_right_id = self.osm.add_way([(c[0], c[1]) for c in new_right.coords], {"type": "line_thin", "subtype": "solid", "width": f"{self.line_width_tag_m:.3f}"})
                if new_left_id and new_right_id:
                    self._set_relation_members(rid, new_left_id, new_right_id)
        # done

    def _rebuild_from_offset(self, rid: str) -> None:
        center = self.lanelet_centers.get(rid)
        if center is None:
            return
        # Estimate local fillet radius from adjacent lanelets in same sequence if possible
        left = self._make_offset_boundary(center, "left")
        right = self._make_offset_boundary(center, "right")
        left_id = self.osm.add_way([(c[0], c[1]) for c in left.coords], {"type": "line_thin", "subtype": "solid", "width": f"{self.line_width_tag_m:.3f}"})
        right_id = self.osm.add_way([(c[0], c[1]) for c in right.coords], {"type": "line_thin", "subtype": "solid", "width": f"{self.line_width_tag_m:.3f}"})
        if left_id and right_id:
            self._set_relation_members(rid, left_id, right_id)

    def _build_turn_lane_from_corner(self, prev_id: str, turn_id: str, next_id: str) -> None:
        ca = self.lanelet_centers.get(prev_id)
        cb = self.lanelet_centers.get(next_id)
        if ca is None or cb is None:
            return
        # Determine turn side via cross product of direction vectors
        va = np.array([ca.coords[-1][0] - ca.coords[-2][0], ca.coords[-1][1] - ca.coords[-2][1]]) if len(ca.coords) >= 2 else np.array([1.0, 0.0])
        vb = np.array([cb.coords[1][0] - cb.coords[0][0], cb.coords[1][1] - cb.coords[0][1]]) if len(cb.coords) >= 2 else np.array([1.0, 0.0])
        crossz = np.cross(va, vb)
        side = "left" if crossz > 0 else "right"
        # Joint midpoint
        ea = Point(ca.coords[-1])
        sb = Point(cb.coords[0])
        mid_guess = Point((ea.x + sb.x) / 2.0, (ea.y + sb.y) / 2.0)
        # Pick apex from sidewalk_area vertices near joint (within 10 m)
        apex = None
        gdf_sw = self.gdf.get("sidewalk_area")
        if gdf_sw is not None and not gdf_sw.empty:
            best_d = 10.0
            for poly in gdf_sw.geometry:
                if not isinstance(poly, Polygon):
                    continue
                for c in list(poly.exterior.coords):
                    x, y = c[0], c[1]
                    p = Point(x, y)
                    d = _geodesic_point_point_distance_m(mid_guess, p)
                    if d < best_d:
                        best_d = d
                        apex = p
        if apex is None:
            apex = mid_guess
        # Inner boundary endpoints: offset inside from prev/end and next/start
        p1c = _interpolate_from_end_m(ca, min(3.0, _line_length_m(ca) * 0.2))
        p2c = _interpolate_from_start_m(cb, min(3.0, _line_length_m(cb) * 0.2))
        hdg1 = _heading_between_points_deg(_interpolate_from_end_m(ca, min(3.2, _line_length_m(ca) * 0.22)), p1c)
        hdg2 = _heading_between_points_deg(p2c, _interpolate_from_start_m(cb, min(3.2, _line_length_m(cb) * 0.22)))
        sign = 1.0 if side == "left" else -1.0
        p1 = _offset_point(p1c, hdg1, sign * self.lane_width_m / 2.0)
        p2 = _offset_point(p2c, hdg2, sign * self.lane_width_m / 2.0)
        # Circle through (p1, apex, p2)
        circ = _circle_from_three_points(p1, apex, p2)
        inner_pts: List[Tuple[float, float]]
        if circ is not None:
            center, _r = circ
            clockwise = np.cross(
                np.array([p1.x - center.x, p1.y - center.y]),
                np.array([p2.x - center.x, p2.y - center.y])
            ) < 0
            inner_pts = _arc_points(center, p1, p2, clockwise, step_deg=6.0)
        else:
            inner_pts = [(p1.x, p1.y), (apex.x, apex.y), (p2.x, p2.y)]
        # Outer boundary by offsetting inner along local tangents
        outer_pts: List[Tuple[float, float]] = []
        for k in range(len(inner_pts)):
            if k == 0:
                pa = Point(inner_pts[k])
                pb = Point(inner_pts[k + 1])
            else:
                pa = Point(inner_pts[k - 1])
                pb = Point(inner_pts[k])
            hdg = _heading_between_points_deg(pa, pb)
            q = _offset_point(Point(inner_pts[k]), hdg, -sign * self.lane_width_m)
            outer_pts.append((q.x, q.y))
        # Update relation members
        left_geom_pts = inner_pts if side == "left" else outer_pts
        right_geom_pts = outer_pts if side == "left" else inner_pts
        left_id = self.osm.add_way(left_geom_pts, {"type": "line_thin", "subtype": "solid", "width": f"{self.line_width_tag_m:.3f}"})
        right_id = self.osm.add_way(right_geom_pts, {"type": "line_thin", "subtype": "solid", "width": f"{self.line_width_tag_m:.3f}"})
        if left_id and right_id:
            self._set_relation_members(turn_id, left_id, right_id)

    def refine_intersections_by_sequences(self) -> None:
        if not self.refine_sequences:
            return
        for seq in self.refine_sequences:
            ids = [str(x) for x in seq]
            # Snap consecutive endpoints and rebuild boundaries for middle lanes via offset
            for a, b in zip(ids[:-1], ids[1:]):
                ca = self.lanelet_centers.get(a)
                cb = self.lanelet_centers.get(b)
                if ca is None or cb is None:
                    continue
                ea = Point(ca.coords[-1])
                sb = Point(cb.coords[0])
                # heading alignment around joint
                ha = _heading_between_points_deg(Point(ca.coords[-2]), ea) if len(ca.coords) >= 2 else _heading_deg(ca)
                hb = _heading_between_points_deg(sb, Point(cb.coords[1])) if len(cb.coords) >= 2 else _heading_deg(cb)
                # derive joint point by short-chord circular fit
                la = _interpolate_from_end_m(ca, min(3.0, _line_length_m(ca) * 0.2))
                lb = _interpolate_from_start_m(cb, min(3.0, _line_length_m(cb) * 0.2))
                mid_guess = Point((ea.x + sb.x) / 2.0, (ea.y + sb.y) / 2.0)
                circ = _circle_from_three_points(la, mid_guess, lb)
                if circ is not None:
                    ccenter, _ = circ
                    clockwise = np.cross(
                        np.array([la.x - ccenter.x, la.y - ccenter.y]),
                        np.array([lb.x - ccenter.x, lb.y - ccenter.y])
                    ) < 0
                    arc_pts = _arc_points(ccenter, la, lb, clockwise, step_deg=10.0)
                    mid = Point(arc_pts[len(arc_pts)//2][0], arc_pts[len(arc_pts)//2][1])
                else:
                    mid = mid_guess
                # apply snap and remove tiny back-steps
                ca_coords = [(c[0], c[1]) for c in ca.coords]
                ca_coords[-1] = (mid.x, mid.y)
                if len(ca_coords) >= 3 and _geodesic_point_point_distance_m(Point(*ca_coords[-2]), Point(*ca_coords[-1])) < 0.2:
                    ca_coords.pop(-2)
                self.lanelet_centers[a] = LineString(ca_coords)
                cb_coords = [(c[0], c[1]) for c in cb.coords]
                cb_coords[0] = (mid.x, mid.y)
                if len(cb_coords) >= 3 and _geodesic_point_point_distance_m(Point(*cb_coords[0]), Point(*cb_coords[1])) < 0.2:
                    cb_coords.pop(1)
                self.lanelet_centers[b] = LineString(cb_coords)
            # Rebuild boundaries: if triple or more, apply corner-aware for middle items
            if len(ids) >= 3:
                for i in range(1, len(ids) - 1):
                    self._build_turn_lane_from_corner(ids[i - 1], ids[i], ids[i + 1])
            else:
                for rid in ids:
                    self._rebuild_from_offset(rid)
            # Force successors/predecessors along the sequence
            for a, b in zip(ids[:-1], ids[1:]):
                self.manual_successors.setdefault(a, [])
                if b not in self.manual_successors[a]:
                    self.manual_successors[a].append(b)
                self.manual_predecessors.setdefault(b, [])
                if a not in self.manual_predecessors[b]:
                    self.manual_predecessors[b].append(a)

    # --------------------- Ancillary features ---------------------
    def add_stop_lines(self) -> None:
        gdf = self.gdf.get("stopline")
        if gdf is None:
            return
        for geom in gdf.geometry:
            if isinstance(geom, LineString) and _line_length_m(geom) > 0.2:
                self.osm.add_way(_coords_xy(geom), {"type": "stop_line"})

    def add_road_markings(self) -> None:
        gdf = self.gdf.get("road_marking_lines")
        if gdf is None:
            return
        for geom in gdf.geometry:
            if isinstance(geom, LineString) and _line_length_m(geom) > 0.2:
                self.osm.add_way(_coords_xy(geom), {"type": "road_marking"})

    def add_crosswalks(self) -> None:
        gdf = self.gdf.get("sidewalk_area")
        if gdf is None:
            return
        for poly in gdf.geometry:
            if not isinstance(poly, Polygon) or poly.is_empty:
                continue
            rect = poly.minimum_rotated_rectangle
            if not isinstance(rect, Polygon):
                continue
            coords = list(rect.exterior.coords)[:-1]
            if len(coords) != 4:
                continue
            # Identify two long edges
            edges = [LineString([coords[i], coords[(i + 1) % 4]]) for i in range(4)]
            lengths = [_line_length_m(e) for e in edges]
            idx = np.argsort(lengths)
            long1, long2 = edges[idx[-1]], edges[idx[-2]]
            # Build ways and relation
            left_id = self.osm.add_way(_coords_xy(long1), {"type": "line_thin", "subtype": "solid", "width": f"{self.line_width_tag_m:.3f}"})
            right_id = self.osm.add_way(_coords_xy(long2), {"type": "line_thin", "subtype": "solid", "width": f"{self.line_width_tag_m:.3f}"})
            if not left_id or not right_id:
                continue
            rel_tags = {
                "type": "lanelet",
                "subtype": "crosswalk",
                "participant:pedestrian": "yes",
                "speed_limit": str(DEFAULT_SPEED_LIMIT),
            }
            rel_id = self.osm.add_relation(
                members=[("way", left_id, "left"), ("way", right_id, "right")],
                tags=rel_tags,
            )
            # store artificial center for adjacency: line between midpoints of long edges
            mid1 = long1.interpolate(0.5, normalized=True)
            mid2 = long2.interpolate(0.5, normalized=True)
            center = LineString([(mid1.x, mid1.y), (mid2.x, mid2.y)])
            self.lanelet_relations.append(rel_id)
            self.lanelet_centers[rel_id] = center

    def add_traffic_lights(self) -> None:
        gdf = self.gdf.get("traffic_location")
        if gdf is None:
            return
        for pt in gdf.geometry:
            if not isinstance(pt, Point):
                continue
            # traffic_light way (single node way)
            tl_way_id = self.osm.add_way([(pt.x, pt.y), (pt.x, pt.y)], {"type": "traffic_light", "subtype": "red_yellow_green", "height": "0.450000"})
            # light_bulbs referencing the traffic light id
            if tl_way_id:
                self.osm.add_way([(pt.x, pt.y)], {"type": "light_bulbs", "traffic_light_id": tl_way_id})

    # --------------------- Adjacency (successors/predecessors) ---------------------
    def build_adjacency(self) -> Dict[str, Dict[str, List[str]]]:
        # Represent lanelet by centerline start/end points and heading
        lane_data: Dict[str, Tuple[Point, Point, float]] = {}
        for rid, center in self.lanelet_centers.items():
            if not isinstance(center, LineString) or len(center.coords) < 2:
                # degenerate, use centroid duplicated
                c = center.centroid if isinstance(center, LineString) else Point(0.0, 0.0)
                lane_data[rid] = (c, c, 0.0)
            else:
                s = Point(center.coords[0])
                e = Point(center.coords[-1])
                lane_data[rid] = (s, e, _heading_deg(center))

        rel_ids: List[str] = list(lane_data.keys())
        adjacency: Dict[str, Dict[str, List[str]]] = {rid: {"successors": [], "predecessors": []} for rid in rel_ids}

        for i, rid in enumerate(rel_ids):
            s_i, e_i, hdg_i = lane_data[rid]
            # only connect road->road (crosswalk 등은 제외)
            rel_i = self.osm.relations.get(rid)
            if rel_i and rel_i.tags.get("subtype") != "road":
                continue
            # Search candidates whose start is near our end
            for j, rid_j in enumerate(rel_ids):
                if i == j:
                    continue
                s_j, _, hdg_j = lane_data[rid_j]
                # only connect to road lanelets as successors
                rel_j = self.osm.relations.get(rid_j)
                if rel_j and rel_j.tags.get("subtype") != "road":
                    continue
                # distance constraint (point-to-point)
                d_m = _geodesic_point_point_distance_m(e_i, s_j)
                if d_m > max(SUCCESSOR_SEARCH_RADIUS_M, INTERSECTION_SNAP_RADIUS_M):
                    continue
                # direction constraint
                dir_thresh = SUCCESSOR_DIR_THRESHOLD_DEG
                # relax direction if both near a snapped cluster center
                relax = False
                for c in self.endpoint_clusters:
                    if _geodesic_point_point_distance_m(e_i, c) <= INTERSECTION_SNAP_RADIUS_M and _geodesic_point_point_distance_m(s_j, c) <= INTERSECTION_SNAP_RADIUS_M:
                        relax = True
                        break
                if relax:
                    dir_thresh = INTERSECTION_DIR_THRESHOLD_DEG
                if _angle_diff_deg(hdg_i, hdg_j) > dir_thresh:
                    continue
                # vector alignment: end_i -> start_j must align with hdg_i
                ang1 = _angle_between_heading_and_vector_deg(hdg_i, e_i, s_j)
                if ang1 > SUCCESSOR_VEC_ALIGN_DEG:
                    continue
                # lateral offset constraint: start_j should be close to forward extension, not to the side
                lat_off = abs(_signed_lateral_offset_m(hdg_i, e_i, s_j))
                if lat_off > self.lane_width_m * LATERAL_OFFSET_TOL_RATIO:
                    continue
                adjacency[rid]["successors"].append(rid_j)
                adjacency[rid_j]["predecessors"].append(rid)

        # Write tags into relations
        for rid, rel in self.osm.relations.items():
            if rid in adjacency:
                # merge manual overrides
                if rid in self.manual_successors:
                    for sid in self.manual_successors[rid]:
                        if sid not in adjacency[rid]["successors"]:
                            adjacency[rid]["successors"].append(sid)
                if rid in self.manual_predecessors:
                    for pid in self.manual_predecessors[rid]:
                        if pid not in adjacency[rid]["predecessors"]:
                            adjacency[rid]["predecessors"].append(pid)
                succ = ";".join(adjacency[rid]["successors"]) if adjacency[rid]["successors"] else ""
                pred = ";".join(adjacency[rid]["predecessors"]) if adjacency[rid]["predecessors"] else ""
                if succ:
                    rel.tags["successors"] = succ
                if pred:
                    rel.tags["predecessors"] = pred
        return adjacency

    # --------------------- Map projector & config ---------------------
    def _compute_center_latlon(self) -> Optional[Tuple[float, float]]:
        if self.osm.nodes:
            lats = [lat for (_, lat) in self.osm.nodes.values()]
            lons = [lon for (lon, _) in self.osm.nodes.values()]
            return (sum(lats) / len(lats), sum(lons) / len(lons))
        return None

    def write_projector_and_config(self) -> None:
        center = self._compute_center_latlon()
        if center is None:
            return
        lat, lon = center

        if self.output_projector_yaml:
            grid = None
            if mgrs is not None:
                m = mgrs.MGRS()
                grid = m.toMGRS(lat, lon)[:5]
            else:
                # Fallback: leave empty grid if mgrs is not available
                grid = ""
            content = f"mgrs_grid: {grid}\nprojector_type: MGRS\nvertical_datum: WGS84\n"
            with open(self.output_projector_yaml, "w", encoding="utf-8") as f:
                f.write(content)

        if self.output_map_config_yaml:
            data = {
                "/**": {
                    "ros__parameters": {
                        "map_origin": {
                            "latitude": float(lat),
                            "longitude": float(lon),
                            "elevation": 0.0,
                            "roll": 0.0,
                            "pitch": 0.0,
                            "yaw": 0.0,
                        }
                    }
                }
            }
            import yaml

            with open(self.output_map_config_yaml, "w", encoding="utf-8") as f:
                yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)

    # --------------------- Orchestration ---------------------
    def run(self) -> None:
        self.load_layers()
        self.segment_links()
        self.build_lanelets()
        self.fix_lanelet_orientation_and_width()
        # Optional targeted refinement
        self.refine_intersections_by_sequences()
        self.add_stop_lines()
        self.add_road_markings()
        self.add_crosswalks()
        self.add_traffic_lights()

        adjacency = self.build_adjacency()

        # Save OSM
        self.osm.save_osm(self.output_osm)

        # Save adjacency JSON next to OSM
        adj_json_path = os.path.splitext(self.output_osm)[0] + "_adjacency.json"
        with open(adj_json_path, "w", encoding="utf-8") as f:
            json.dump(adjacency, f, indent=2)

        # Projector and config
        self.write_projector_and_config()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert KCity V3 shapefiles to Lanelet2 OSM (sample-compatible tags)")
    parser.add_argument("--input_dir", required=True, help="Path to input directory containing shapefiles (kcity_v3_all)")
    parser.add_argument("--output_osm", required=True, help="Path to output OSM file")
    parser.add_argument("--output_projector_yaml", default=None, help="Path to output map_projector_info.yaml")
    parser.add_argument("--output_map_config_yaml", default=None, help="Path to output map_config.yaml")
    parser.add_argument("--lane_width", type=float, default=DEFAULT_LANE_WIDTH_M, help="Lane width in meters")
    parser.add_argument("--line_width_tag", type=float, default=DEFAULT_LINE_WIDTH_TAG_M, help="Line width tag value in meters")
    parser.add_argument("--refine_sequences", nargs="*", help="Optional lanelet id sequences to refine, e.g., '8-29-23 39-34-8' (space-separated)")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    seqs: Optional[List[List[str]]] = None
    if args.refine_sequences:
        seqs = []
        for token in args.refine_sequences:
            parts = [p.strip() for p in token.split("-") if p.strip()]
            if len(parts) >= 2:
                seqs.append(parts)
    converter = KCityV3Converter(
        input_dir=args.input_dir,
        output_osm=args.output_osm,
        output_projector_yaml=args.output_projector_yaml,
        output_map_config_yaml=args.output_map_config_yaml,
        lane_width_m=args.lane_width,
        line_width_tag_m=args.line_width_tag,
        refine_sequences=seqs,
    )
    converter.run()


