import argparse
import json
import math
import os
from collections import defaultdict
from typing import Dict, List, Optional, Tuple

import geopandas as gpd
import numpy as np
from lxml import etree
from shapely.geometry import LineString, Point, Polygon, MultiLineString, box as shp_box
from shapely.ops import linemerge
from shapely.strtree import STRtree


# -------------------- Configuration --------------------

DEFAULT_LANE_WIDTH_M = 3.5
DEFAULT_LINE_WIDTH_TAG_M = 0.200
COORD_ROUND_DECIMALS = 7

BOUNDARY_SAMPLE_STEP_M = 4.0
BOUNDARY_PARALLEL_THRESH_DEG = 85.0
BOUNDARY_MIN_OFFSET_M = 0.05
BOUNDARY_MAX_OFFSET_M_FACTOR = 12.0
LEFT_RIGHT_MIN_VOTES = 4
MIN_SAMPLE_VOTE_RATIO = 0.1

# Voting parameters for pair selection (pure geometry)
PAIR_SEARCH_RADIUS_FACTOR = 8.0  # x lane_width
PAIR_MIN_VOTES = 2
PAIR_EQUIDIST_RATIO_MIN = 0.3
PAIR_EQUIDIST_RATIO_MAX = 3.0

STOPLINE_MIN_LENGTH_M = 0.3

SUCCESSOR_SNAP_RADIUS_M = 3.0
SUCCESSOR_DIR_THRESHOLD_DEG = 60.0


# -------------------- Small utilities --------------------

def _round_coord(lon: float, lat: float) -> Tuple[float, float]:
    return (round(lon, COORD_ROUND_DECIMALS), round(lat, COORD_ROUND_DECIMALS))


def _deg_m_scale(lat: float) -> Tuple[float, float]:
    m_per_deg_lat = 111111.0
    m_per_deg_lon = 111111.0 * math.cos(math.radians(lat))
    return m_per_deg_lon, m_per_deg_lat


def _geodesic_point_point_distance_m(p1: Point, p2: Point) -> float:
    m_per_deg_lon, m_per_deg_lat = _deg_m_scale((p1.y + p2.y) / 2.0)
    dx = (p2.x - p1.x) * m_per_deg_lon
    dy = (p2.y - p1.y) * m_per_deg_lat
    return math.hypot(dx, dy)


def _line_length_m(line: LineString) -> float:
    if line.is_empty:
        return 0.0
    total = 0.0
    coords = [(c[0], c[1]) for c in line.coords]
    for (x1, y1), (x2, y2) in zip(coords[:-1], coords[1:]):
        total += _geodesic_point_point_distance_m(Point(x1, y1), Point(x2, y2))
    return total


def _heading_deg(line: LineString) -> float:
    coords = [(c[0], c[1]) for c in line.coords]
    if len(coords) < 2:
        return 0.0
    (x1, y1), (x2, y2) = coords[0], coords[-1]
    m_per_deg_lon, m_per_deg_lat = _deg_m_scale((y1 + y2) / 2.0)
    dx = (x2 - x1) * m_per_deg_lon
    dy = (y2 - y1) * m_per_deg_lat
    ang = math.degrees(math.atan2(dy, dx))
    return (ang + 360.0) % 360.0


def _angle_diff_deg(a: float, b: float) -> float:
    d = abs((a - b + 180.0) % 360.0 - 180.0)
    return d


def _sample_line_by_distance(line: LineString, step_m: float) -> List[Tuple[float, float]]:
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


def _signed_lateral_offset_m(ref_heading_deg: float, origin: Point, target: Point) -> float:
    m_per_deg_lon, m_per_deg_lat = _deg_m_scale((origin.y + target.y) / 2.0)
    vec = np.array([
        (target.x - origin.x) * m_per_deg_lon,
        (target.y - origin.y) * m_per_deg_lat,
    ])
    hdg = math.radians(ref_heading_deg)
    fwd = np.array([math.cos(hdg), math.sin(hdg)])
    left = np.array([-fwd[1], fwd[0]])
    return float(np.dot(vec, left))


def _coords_xy(geom: LineString) -> List[Tuple[float, float]]:
    return [(c[0], c[1]) for c in geom.coords]


def _norm_link_value(v: object) -> str:
    # Normalize link identifiers to comparable canonical strings.
    # Handles None/NaN, ints, floats like 12.0 -> "12", and strips whitespace for strings.
    try:
        import math as _math
        import numpy as _np
    except Exception:
        pass
    if v is None:
        return ""
    # pandas NaN or numpy.nan
    try:
        if isinstance(v, float) and _math.isnan(v):
            return ""
    except Exception:
        pass
    try:
        import numpy as _np
        if isinstance(v, _np.floating) and _np.isnan(v):
            return ""
    except Exception:
        pass
    # Numeric normalization
    try:
        # Attempt float parse
        f = float(v)
        i = int(round(f))
        if abs(f - i) < 1e-9:
            return str(i)
        return str(f)
    except Exception:
        s = str(v).strip()
        # If it is like '12.0' make it '12'
        try:
            f2 = float(s)
            i2 = int(round(f2))
            if abs(f2 - i2) < 1e-9:
                return str(i2)
            return str(f2)
        except Exception:
            return s


# -------------------- OSM builder --------------------

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
        self.ways: Dict[str, Dict] = {}
        self.relations: Dict[str, Dict] = {}
        self.ids = IdGenerator()

    def add_node(self, lon: float, lat: float) -> str:
        coord = _round_coord(lon, lat)
        if coord in self.node_coord_to_id:
            return self.node_coord_to_id[coord]
        nid = self.ids.next_node()
        self.nodes[nid] = coord
        self.node_coord_to_id[coord] = nid
        return nid

    def add_way(self, coords: List[Tuple[float, float]], tags: Optional[Dict[str, str]] = None) -> Optional[str]:
        if not coords:
            return None
        # Deduplicate consecutive
        dedup: List[Tuple[float, float]] = [coords[0]]
        for (x, y) in coords[1:]:
            r = _round_coord(x, y)
            if r != _round_coord(*dedup[-1]):
                dedup.append((x, y))
        if len(dedup) < 2:
            return None
        node_refs = [self.add_node(x, y) for (x, y) in dedup]
        wid = self.ids.next_way()
        self.ways[wid] = {"nodes": node_refs, "tags": tags or {}}
        return wid

    def add_relation(self, members: List[Tuple[str, str, str]], tags: Optional[Dict[str, str]] = None) -> str:
        rid = self.ids.next_relation()
        # Ensure uniqueness if a fixed-id relation already used this id
        while str(rid) in self.relations:
            rid = self.ids.next_relation()
        self.relations[rid] = {"members": members, "tags": tags or {}}
        return rid

    def add_relation_with_id(self, relation_id: str, members: List[Tuple[str, str, str]], tags: Optional[Dict[str, str]] = None) -> str:
        # Use a fixed relation id (e.g., Link value) as requested
        rid = str(relation_id)
        self.relations[rid] = {"members": members, "tags": tags or {}}
        return rid

    def save(self, output_path: str) -> None:
        root = etree.Element("osm", version="0.6", generator="KCityV5ToLanelet2")
        for nid, (lon, lat) in self.nodes.items():
            etree.SubElement(root, "node", id=nid, lat=str(lat), lon=str(lon))
        for wid, w in self.ways.items():
            we = etree.SubElement(root, "way", id=wid)
            for ref in w["nodes"]:
                etree.SubElement(we, "nd", ref=ref)
            for k, v in w["tags"].items():
                etree.SubElement(we, "tag", k=str(k), v=str(v))
        for rid, r in self.relations.items():
            re = etree.SubElement(root, "relation", id=rid)
            for t, ref, role in r["members"]:
                etree.SubElement(re, "member", type=t, ref=ref, role=role)
            for k, v in r["tags"].items():
                etree.SubElement(re, "tag", k=str(k), v=str(v))
        xml = etree.tostring(root, pretty_print=True, xml_declaration=True, encoding="UTF-8")
        with open(output_path, "wb") as f:
            f.write(xml)


# -------------------- Core converter --------------------

class KCityV5Converter:
    def __init__(self, input_dir: str, output_osm: str, lane_width_m: float = DEFAULT_LANE_WIDTH_M, line_width_tag_m: float = DEFAULT_LINE_WIDTH_TAG_M) -> None:
        self.input_dir = input_dir
        self.output_osm = output_osm
        self.lane_width_m = lane_width_m
        self.line_width_tag_m = line_width_tag_m
        self.osm = OsmBuilder()
        self.gdf: Dict[str, gpd.GeoDataFrame] = {}
        self.centerlines: List[LineString] = []

    def _load(self, name: str) -> Optional[gpd.GeoDataFrame]:
        path = os.path.join(self.input_dir, f"{name}.shp")
        if not os.path.exists(path):
            return None
        gdf = gpd.read_file(path)
        if gdf.crs is not None and gdf.crs.to_epsg() != 4326:
            gdf = gdf.to_crs("EPSG:4326")
        # Keep only valid geometries
        gdf = gdf[~gdf.geometry.is_empty & gdf.geometry.is_valid]
        return gdf

    def load_layers(self) -> None:
        for name in ["path", "LR_for_path", "stopline", "sidewalk"]:
            gdf = self._load(name)
            if gdf is not None and not gdf.empty:
                self.gdf[name] = gdf

        # Centerlines from path
        gdf_path = self.gdf.get("path")
        if gdf_path is not None:
            lines: List[LineString] = []
            for geom in gdf_path.geometry:
                if isinstance(geom, LineString):
                    lines.append(geom)
                elif isinstance(geom, MultiLineString):
                    merged = linemerge(geom)
                    if isinstance(merged, LineString):
                        lines.append(merged)
                    else:
                        lines.extend(list(merged.geoms))
            self.centerlines = [ln for ln in lines if _line_length_m(ln) > 0.5]

    # --- Boundary matching (choose two best LR_for_path lines per centerline) ---
    def _prepare_lr_candidates(self) -> Tuple[List[LineString], Optional[STRtree], Dict[int, int], Dict[str, List[int]], Dict[str, List[int]]]:
        lr_gdf = self.gdf.get("LR_for_path")
        if lr_gdf is None or lr_gdf.empty:
            return [], None, {}, {}, {}
        lines: List[LineString] = []
        # Maintain index mapping to attributes for left/right linkage
        left_map: Dict[str, List[int]] = defaultdict(list)
        right_map: Dict[str, List[int]] = defaultdict(list)
        # Map by boundary ID for direct lookup via path's L_LinkID/R_LinkID
        self.lr_id_to_index: Dict[str, int] = {}
        for geom in lr_gdf.geometry:
            if isinstance(geom, LineString):
                # flatten to 2D if has Z
                try:
                    if hasattr(geom, "has_z") and geom.has_z:
                        geom = LineString([(c[0], c[1]) for c in geom.coords])
                except Exception:
                    pass
                lines.append(geom)
            elif isinstance(geom, MultiLineString):
                for g in geom.geoms:
                    if isinstance(g, LineString):
                        try:
                            if hasattr(g, "has_z") and g.has_z:
                                g = LineString([(c[0], c[1]) for c in g.coords])
                        except Exception:
                            pass
                        lines.append(g)
        tree = STRtree(lines) if lines else None
        id_map: Dict[int, int] = {id(g): i for i, g in enumerate(lines)}
        # Build attribute linkage maps (values coerced to str for matching)
        # Priority 1: New schema - LeftLink (left boundary), RightLink (right boundary)
        if "LeftLink" in lr_gdf.columns or "RightLink" in lr_gdf.columns:
            if "LeftLink" in lr_gdf.columns:
                for idx, val in enumerate(lr_gdf["LeftLink"]):
                    s = _norm_link_value(val)
                    if s:
                        left_map[s].append(idx)
            if "RightLink" in lr_gdf.columns:
                for idx, val in enumerate(lr_gdf["RightLink"]):
                    s = _norm_link_value(val)
                    if s:
                        right_map[s].append(idx)
        else:
            # Fallback: legacy schema R_linkID/L_linkID
            if "R_linkID" in lr_gdf.columns:
                for idx, val in enumerate(lr_gdf["R_linkID"].astype(str).tolist()):
                    if val and val.lower() != "none":
                        left_map[val].append(idx)
            if "L_linkID" in lr_gdf.columns:
                for idx, val in enumerate(lr_gdf["L_linkID"].astype(str).tolist()):
                    if val and val.lower() != "none":
                        right_map[val].append(idx)
        # Boundary ID -> index (for matching by path's L_LinkID/R_LinkID)
        if "ID" in lr_gdf.columns:
            for i, bid in enumerate(lr_gdf["ID"].astype(str).tolist()):
                if bid and bid.lower() != "none":
                    self.lr_id_to_index[bid] = i
        return lines, tree, id_map, left_map, right_map

    def _select_best_by_distance(self, seg: LineString, candidates: List[LineString], max_offset_m: float) -> Optional[LineString]:
        if not candidates:
            return None
        samples = _sample_line_by_distance(seg, BOUNDARY_SAMPLE_STEP_M)
        best_idx = -1
        best_score = 1e9
        seg_hdg = _heading_deg(seg)
        for i, geom in enumerate(candidates):
            if _angle_diff_deg(seg_hdg, _heading_deg(geom)) > BOUNDARY_PARALLEL_THRESH_DEG:
                continue
            offs: List[float] = []
            ok_votes = 0
            for x, y in samples:
                p = Point(x, y)
                proj = geom.interpolate(geom.project(p))
                off = _geodesic_point_point_distance_m(p, proj)
                if BOUNDARY_MIN_OFFSET_M <= off <= max_offset_m:
                    offs.append(off)
                    ok_votes += 1
            # 후보가 샘플의 일정 비율 이상을 만족해야 함
            if ok_votes < max(2, int(len(samples) * MIN_SAMPLE_VOTE_RATIO)):
                continue
            if not offs:
                continue
            score = float(np.median(offs))
            if score < best_score:
                best_score = score
                best_idx = i
        return candidates[best_idx] if best_idx >= 0 else None

    def _match_lr_for_centerline_attr(self, seg: LineString, link_id: str, lr_lines: List[LineString], left_map: Dict[str, List[int]], right_map: Dict[str, List[int]], max_offset_m: float) -> Tuple[Optional[LineString], Optional[LineString]]:
        # Attribute-driven: prefer exact attribute match without strict geometric filtering
        left_idxs = left_map.get(link_id, [])
        right_idxs = right_map.get(link_id, [])
        left_cands = [lr_lines[i] for i in left_idxs]
        right_cands = [lr_lines[i] for i in right_idxs]
        left: Optional[LineString] = None
        right: Optional[LineString] = None
        # If single candidate, accept directly
        if len(left_cands) == 1:
            left = left_cands[0]
        if len(right_cands) == 1:
            right = right_cands[0]
        # If multiple, pick by permissive distance scoring
        if left is None and left_cands:
            left = self._select_best_by_distance(seg, left_cands, max_offset_m)
        if right is None and right_cands:
            right = self._select_best_by_distance(seg, right_cands, max_offset_m)
        # Final fallback: if still None but candidates exist, pick the first candidate
        if left is None and left_cands:
            left = left_cands[0]
        if right is None and right_cands:
            right = right_cands[0]
        return left, right

    def _match_lr_for_centerline_by_path_ids(self, seg: LineString, path_left_id: str, path_right_id: str, lr_lines: List[LineString], max_offset_m: float) -> Tuple[Optional[LineString], Optional[LineString]]:
        # Direct: path.L_LinkID/R_LinkID refer to LR_for_path.ID
        left = None
        right = None
        if path_left_id and path_left_id in getattr(self, 'lr_id_to_index', {}):
            gi = self.lr_id_to_index[path_left_id]
            left = lr_lines[gi]
        if path_right_id and path_right_id in getattr(self, 'lr_id_to_index', {}):
            gi = self.lr_id_to_index[path_right_id]
            right = lr_lines[gi]
        # Validate offsets roughly; drop if too far
        def _validate(g: Optional[LineString]) -> Optional[LineString]:
            if g is None:
                return None
            samples = _sample_line_by_distance(seg, BOUNDARY_SAMPLE_STEP_M)
            ok = 0
            for x, y in samples:
                p = Point(x, y)
                proj = g.interpolate(g.project(p))
                off = _geodesic_point_point_distance_m(p, proj)
                if BOUNDARY_MIN_OFFSET_M <= off <= max_offset_m:
                    ok += 1
            return g if ok >= max(2, int(len(samples) * MIN_SAMPLE_VOTE_RATIO)) else None
        return _validate(left), _validate(right)

    def _match_lr_for_centerline_geo(self, seg: LineString, lr_lines: List[LineString], lr_tree: Optional[STRtree], max_offset_m: float) -> Tuple[Optional[LineString], Optional[LineString]]:
        if lr_tree is None or not lr_lines:
            return None, None
        samples = _sample_line_by_distance(seg, BOUNDARY_SAMPLE_STEP_M)
        hdg = _heading_deg(seg)
        # candidate index -> offsets by side
        votes: Dict[int, Dict[str, List[float]]] = defaultdict(lambda: {"left": [], "right": []})
        for x, y in samples:
            p = Point(x, y)
            m_per_deg_lon, m_per_deg_lat = _deg_m_scale(y)
            # search radius from expected max offset
            dx = (max_offset_m * 1.8) / m_per_deg_lon
            dy = (max_offset_m * 1.8) / m_per_deg_lat
            hits = lr_tree.query(shp_box(x - dx, y - dy, x + dx, y + dy))
            if not isinstance(hits, list):
                hits = list(hits) if hits is not None else []
            for geom in hits:
                # ignore pathological candidates
                if _angle_diff_deg(hdg, _heading_deg(geom)) > BOUNDARY_PARALLEL_THRESH_DEG:
                    continue
                proj = geom.interpolate(geom.project(p))
                off = _geodesic_point_point_distance_m(p, proj)
                if off < BOUNDARY_MIN_OFFSET_M or off > max_offset_m:
                    continue
                s = _signed_lateral_offset_m(hdg, p, proj)
                idx = None
                # obtain index by identity; fallback to equals search
                try:
                    idx = lr_lines.index(geom)
                except Exception:
                    # slow fallback
                    for k, g in enumerate(lr_lines):
                        if g.equals(geom):
                            idx = k
                            break
                if idx is None:
                    continue
                if s > 0:
                    votes[idx]["left"].append(off)
                elif s < 0:
                    votes[idx]["right"].append(off)

        def pick(side: str) -> Optional[LineString]:
            best_i = -1
            best_score = 1e9
            for i, data in votes.items():
                offs = data.get(side, [])
                if len(offs) < max(2, int(len(samples) * MIN_SAMPLE_VOTE_RATIO)):
                    continue
                score = float(np.median(offs))
                if score < best_score:
                    best_score = score
                    best_i = i
            return lr_lines[best_i] if best_i >= 0 else None

        left = pick("left")
        right = pick("right")
        # avoid selecting the same geometry for both sides
        if left is not None and right is not None and left.equals(right):
            # invalidate the worse side by larger score
            # recompute simple scores
            def score(geom: LineString, side: str) -> float:
                offs: List[float] = []
                for x, y in samples:
                    p = Point(x, y)
                    proj = geom.interpolate(geom.project(p))
                    off = _geodesic_point_point_distance_m(p, proj)
                    if BOUNDARY_MIN_OFFSET_M <= off <= max_offset_m:
                        offs.append(off)
                return float(np.median(offs)) if offs else 1e9
            if score(left, "left") <= score(right, "right"):
                right = None
            else:
                left = None
        return left, right

    def _pair_lr_by_votes(self, seg: LineString, lr_lines: List[LineString], lr_tree: Optional[STRtree]) -> Tuple[Optional[LineString], Optional[LineString]]:
        if lr_tree is None or not lr_lines:
            return None, None
        samples = _sample_line_by_distance(seg, BOUNDARY_SAMPLE_STEP_M)
        hdg = _heading_deg(seg)
        # Build neighborhood for each sample and score candidate pairs by equidistance and side consistency
        pair_votes: Dict[Tuple[int, int], int] = defaultdict(int)
        for x, y in samples:
            p = Point(x, y)
            m_per_deg_lon, m_per_deg_lat = _deg_m_scale(y)
            dx = (self.lane_width_m * PAIR_SEARCH_RADIUS_FACTOR) / m_per_deg_lon
            dy = (self.lane_width_m * PAIR_SEARCH_RADIUS_FACTOR) / m_per_deg_lat
            hits = lr_tree.query(shp_box(x - dx, y - dy, x + dx, y + dy))
            if not isinstance(hits, list):
                hits = list(hits) if hits is not None else []
            # project all candidates and keep the two closest that lie on opposite sides
            cands: List[Tuple[int, float, float]] = []  # (idx, offset_m, side_sign)
            for geom in hits:
                if _angle_diff_deg(hdg, _heading_deg(geom)) > BOUNDARY_PARALLEL_THRESH_DEG:
                    continue
                proj = geom.interpolate(geom.project(p))
                off = _geodesic_point_point_distance_m(p, proj)
                if off < BOUNDARY_MIN_OFFSET_M or off > self.lane_width_m * BOUNDARY_MAX_OFFSET_M_FACTOR:
                    continue
                s = _signed_lateral_offset_m(hdg, p, proj)
                if abs(s) < 1e-6:
                    continue
                # find geometry index
                try:
                    idx = lr_lines.index(geom)
                except Exception:
                    idx = None
                    for k, g in enumerate(lr_lines):
                        if g.equals(geom):
                            idx = k
                            break
                if idx is None:
                    continue
                cands.append((idx, off, 1.0 if s > 0 else -1.0))
            # choose best opposite-side pair by equidistance ratio closeness to 1
            best = None
            best_score = 1e9
            for i in range(len(cands)):
                for j in range(i + 1, len(cands)):
                    a = cands[i]
                    b = cands[j]
                    if a[2] * b[2] >= 0:
                        continue  # same side
                    off_small = min(a[1], b[1])
                    off_large = max(a[1], b[1])
                    ratio = off_large / (off_small + 1e-9)
                    if not (PAIR_EQUIDIST_RATIO_MIN <= ratio <= PAIR_EQUIDIST_RATIO_MAX):
                        continue
                    score = abs(1.0 - ratio)
                    if score < best_score:
                        best_score = score
                        best = (a, b)
            if best is None:
                continue
            # normalize order: (left_idx, right_idx)
            a, b = best
            left_idx = a[0] if a[2] > 0 else b[0]
            right_idx = b[0] if a[2] > 0 else a[0]
            key = (left_idx, right_idx)
            pair_votes[key] += 1
        # choose the pair with max votes
        if not pair_votes:
            return None, None
        best_pair, votes = max(pair_votes.items(), key=lambda kv: kv[1])
        if votes < PAIR_MIN_VOTES:
            return None, None
        left = lr_lines[best_pair[0]]
        right = lr_lines[best_pair[1]]
        return left, right

    def _build_center_from_lr(self, left: LineString, right: LineString, num_samples: int = 25) -> List[Tuple[float, float]]:
        # Generate centerline as midpoint between left/right projections
        pts: List[Tuple[float, float]] = []
        if left.is_empty or right.is_empty:
            return pts
        for i in range(num_samples):
            t = 0.0 if num_samples <= 1 else i / (num_samples - 1)
            lp = left.interpolate(t, normalized=True)
            rp = right.interpolate(right.project(lp))
            mx = (lp.x + rp.x) / 2.0
            my = (lp.y + rp.y) / 2.0
            if not pts or (abs(mx - pts[-1][0]) > 1e-9 or abs(my - pts[-1][1]) > 1e-9):
                pts.append((mx, my))
        return pts

    def build_lanelets(self) -> None:
        lr_lines, lr_tree, lr_id_map, left_map, right_map = self._prepare_lr_candidates()
        gdf_path = self.gdf.get("path")
        if gdf_path is None or gdf_path.empty:
            return
        # Iterate each path centerline with its ID and choose LR using attributes
        # 동적으로 허용 오프셋 상한 설정: 차선 폭의 0.7~5배, 상한 20m
        max_off_m = max(self.lane_width_m * 0.5, min(self.lane_width_m * 12.0, 40.0))
        for _, row in gdf_path.iterrows():
            seg = row.geometry
            # New schema: path 'Link' column for lane matching
            link_raw = row.get("Link", row.get("ID", ""))
            link_id = _norm_link_value(link_raw)
            path_left_id = str(row.get("L_LinkID", "")) if row.get("L_LinkID", "") is not None else ""
            path_right_id = str(row.get("R_LinkID", "")) if row.get("R_LinkID", "") is not None else ""
            if not link_id:
                continue
            if not isinstance(seg, LineString) or seg.is_empty:
                continue
            # ensure we are working in 2D
            try:
                if hasattr(seg, "has_z") and seg.has_z:
                    seg = LineString([(c[0], c[1]) for c in seg.coords])
            except Exception:
                pass
            # Attribute-first: by Link == LeftLink/RightLink (strict: pick exactly those equal to Link)
            left = None
            right = None
            # candidates equal to Link only
            l_idx_list = left_map.get(link_id, [])
            r_idx_list = right_map.get(link_id, [])
            if len(l_idx_list) == 1:
                left = lr_lines[l_idx_list[0]]
            elif len(l_idx_list) > 1:
                left = self._select_best_by_distance(seg, [lr_lines[i] for i in l_idx_list], max_off_m)
            if len(r_idx_list) == 1:
                right = lr_lines[r_idx_list[0]]
            elif len(r_idx_list) > 1:
                right = self._select_best_by_distance(seg, [lr_lines[i] for i in r_idx_list], max_off_m)
            # If missing, try pure geometry as fallback (pair voting -> per-side geometric voting)
            if left is None or right is None:
                pleft, pright = self._pair_lr_by_votes(seg, lr_lines, lr_tree)
                left = left or pleft
                right = right or pright
            if left is None or right is None:
                gleft, gright = self._match_lr_for_centerline_geo(seg, lr_lines, lr_tree, max_off_m)
                left = left or gleft
                right = right or gright
            # Strict mode: do not synthesize boundaries; skip if not both found
            if left is None or right is None:
                continue
            # Orientation check (robust): vote across multiple samples
            hdg = _heading_deg(seg)
            samples = _sample_line_by_distance(seg, max(BOUNDARY_SAMPLE_STEP_M, 3.0))
            def vote_ok(lw: LineString, rw: LineString) -> float:
                ok = 0
                tot = 0
                for x, y in samples:
                    p = Point(x, y)
                    lp = lw.interpolate(lw.project(p))
                    rp = rw.interpolate(rw.project(p))
                    sl = _signed_lateral_offset_m(hdg, p, Point(lp.x, lp.y))
                    sr = _signed_lateral_offset_m(hdg, p, Point(rp.x, rp.y))
                    if abs(sl) < 1e-9 or abs(sr) < 1e-9:
                        continue
                    if sl > 0 and sr < 0:
                        ok += 1
                    tot += 1
                return (ok / tot) if tot > 0 else 0.0
            good = vote_ok(left, right)
            if good < 0.6:
                alt = vote_ok(right, left)
                if alt > good:
                    left, right = right, left

            left_id = self.osm.add_way(_coords_xy(left), {"type": "line_thin", "subtype": "solid", "width": f"{self.line_width_tag_m:.3f}"})
            right_id = self.osm.add_way(_coords_xy(right), {"type": "line_thin", "subtype": "solid", "width": f"{self.line_width_tag_m:.3f}"})
            # Build centerline from LR midpoints for better consistency
            center_pts = self._build_center_from_lr(left, right, num_samples=max(25, int(_line_length_m(seg) // 2)))
            center_id = self.osm.add_way(center_pts if center_pts else _coords_xy(seg), {})
            if not left_id or not right_id:
                continue
            rel_tags = {"type": "lanelet", "subtype": "road", "id": link_id, "one_way": "yes", "location": "urban", "speed_limit": "30"}
            # Use Link as relation id
            members = [("way", left_id, "left"), ("way", right_id, "right")]
            if center_id:
                members.append(("way", center_id, "centerline"))
            self.osm.add_relation_with_id(link_id, members, rel_tags)

    def add_stoplines(self) -> None:
        gdf = self.gdf.get("stopline")
        if gdf is None or gdf.empty:
            return
        for geom in gdf.geometry:
            if isinstance(geom, LineString) and _line_length_m(geom) >= STOPLINE_MIN_LENGTH_M:
                self.osm.add_way(_coords_xy(geom), {"type": "stop_line"})

    def add_crosswalks(self) -> None:
        gdf = self.gdf.get("sidewalk")
        if gdf is None or gdf.empty:
            return
        # Handle polygons (preferred) and fallback to lines pairing
        polys: List[Polygon] = []
        lines: List[LineString] = []
        for geom in gdf.geometry:
            if isinstance(geom, Polygon):
                polys.append(geom)
            elif isinstance(geom, MultiLineString):
                for g in geom.geoms:
                    if isinstance(g, LineString):
                        lines.append(g)
            elif isinstance(geom, LineString):
                lines.append(geom)

        # Polygon-based crosswalks: take two long edges of minimum rotated rectangle
        for poly in polys:
            rect = poly.minimum_rotated_rectangle
            if not isinstance(rect, Polygon):
                continue
            coords = list(rect.exterior.coords)[:-1]
            if len(coords) != 4:
                continue
            edges = [LineString([coords[i], coords[(i + 1) % 4]]) for i in range(4)]
            lengths = [_line_length_m(e) for e in edges]
            idx = np.argsort(lengths)
            long1, long2 = edges[idx[-1]], edges[idx[-2]]
            l_id = self.osm.add_way(_coords_xy(long1), {"type": "line_thin", "subtype": "solid", "width": f"{self.line_width_tag_m:.3f}"})
            r_id = self.osm.add_way(_coords_xy(long2), {"type": "line_thin", "subtype": "solid", "width": f"{self.line_width_tag_m:.3f}"})
            if l_id and r_id:
                self.osm.add_relation([("way", l_id, "left"), ("way", r_id, "right")], {"type": "lanelet", "subtype": "crosswalk", "participant:pedestrian": "yes"})

        # Line-based fallback: greedily pair two near-parallel lines within ~6 m
        used = [False] * len(lines)
        for i, a in enumerate(lines):
            if used[i]:
                continue
            best_j = -1
            best_w = 1e9
            ha = _heading_deg(a)
            for j, b in enumerate(lines):
                if i == j or used[j]:
                    continue
                if _angle_diff_deg(ha, _heading_deg(b)) > 25.0:
                    continue
                # width ~ median distance between midpoints projections
                ma = a.interpolate(0.5, normalized=True)
                mb = b.interpolate(0.5, normalized=True)
                w = _geodesic_point_point_distance_m(Point(ma.x, ma.y), Point(mb.x, mb.y))
                if 1.0 <= w <= 8.0 and w < best_w:
                    best_w = w
                    best_j = j
            if best_j >= 0:
                used[i] = used[best_j] = True
                l_id = self.osm.add_way(_coords_xy(lines[i]), {"type": "line_thin", "subtype": "solid", "width": f"{self.line_width_tag_m:.3f}"})
                r_id = self.osm.add_way(_coords_xy(lines[best_j]), {"type": "line_thin", "subtype": "solid", "width": f"{self.line_width_tag_m:.3f}"})
                if l_id and r_id:
                    self.osm.add_relation([("way", l_id, "left"), ("way", r_id, "right")], {"type": "lanelet", "subtype": "crosswalk", "participant:pedestrian": "yes"})

    def build_adjacency(self) -> Dict[str, Dict[str, List[str]]]:
        # Simple adjacency by endpoint proximity and heading
        lane_rels = [(rid, rel) for rid, rel in self.osm.relations.items() if rel["tags"].get("subtype") == "road"]
        # reconstruct center from left/right midpoints
        centers: Dict[str, Tuple[Point, Point, float]] = {}
        for rid, rel in lane_rels:
            left_way = next((ref for t, ref, role in rel["members"] if role == "left"), None)
            right_way = next((ref for t, ref, role in rel["members"] if role == "right"), None)
            if not left_way or not right_way:
                continue
            lw = self.osm.ways[left_way]
            rw = self.osm.ways[right_way]
            def to_line(way_nodes: List[str]) -> LineString:
                coords = [(self.osm.nodes[n][0], self.osm.nodes[n][1]) for n in way_nodes]
                return LineString(coords)
            lgeom = to_line(lw["nodes"])
            rgeom = to_line(rw["nodes"])
            lm = lgeom.interpolate(0.5, normalized=True)
            rm = rgeom.interpolate(0.5, normalized=True)
            s = Point(lgeom.coords[0][0], lgeom.coords[0][1])
            e = Point(lgeom.coords[-1][0], lgeom.coords[-1][1])
            centers[rid] = (Point((lm.x + rm.x) / 2.0, (lm.y + rm.y) / 2.0), Point((s.x + rgeom.coords[0][0]) / 2.0, (s.y + rgeom.coords[0][1]) / 2.0), _heading_deg(lgeom))

        ids = [rid for rid, _ in lane_rels]
        adj: Dict[str, Dict[str, List[str]]] = {rid: {"successors": [], "predecessors": []} for rid in ids}
        # Use start/end from left geometry approximation
        endpoints: Dict[str, Tuple[Point, Point, float]] = {}
        for rid, rel in lane_rels:
            left_way = next((ref for t, ref, role in rel["members"] if role == "left"), None)
            lw = self.osm.ways.get(left_way)
            if not lw:
                continue
            coords = [(self.osm.nodes[n][0], self.osm.nodes[n][1]) for n in lw["nodes"]]
            s = Point(coords[0][0], coords[0][1])
            e = Point(coords[-1][0], coords[-1][1])
            endpoints[rid] = (s, e, _heading_deg(LineString(coords)))

        for a in ids:
            if a not in endpoints:
                continue
            _, ea, ha = endpoints[a]
            for b in ids:
                if a == b or b not in endpoints:
                    continue
                sb, _, hb = endpoints[b]
                d = _geodesic_point_point_distance_m(ea, sb)
                if d > SUCCESSOR_SNAP_RADIUS_M:
                    continue
                if _angle_diff_deg(ha, hb) > SUCCESSOR_DIR_THRESHOLD_DEG:
                    continue
                adj[a]["successors"].append(b)
                adj[b]["predecessors"].append(a)

        # Write into relation tags
        for rid, rel in self.osm.relations.items():
            if rid in adj:
                succ = ";".join(adj[rid]["successors"]) if adj[rid]["successors"] else ""
                pred = ";".join(adj[rid]["predecessors"]) if adj[rid]["predecessors"] else ""
                if succ:
                    rel["tags"]["successors"] = succ
                if pred:
                    rel["tags"]["predecessors"] = pred
        return adj

    def run(self) -> None:
        self.load_layers()
        self.build_lanelets()
        self.add_stoplines()
        self.add_crosswalks()
        adj = self.build_adjacency()
        # Save OSM and adjacency sidecar
        self.osm.save(self.output_osm)
        adj_json_path = os.path.splitext(self.output_osm)[0] + "_adjacency.json"
        try:
            with open(adj_json_path, "w", encoding="utf-8") as f:
                json.dump(adj, f, indent=2)
        except Exception:
            pass


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Convert kcity_v5 shapefiles (path, LR_for_path, stopline, sidewalk) to Lanelet2 OSM")
    p.add_argument("--input_dir", required=True, help="Directory containing kcity_v5 shapefiles")
    p.add_argument("--output_osm", required=True, help="Output OSM path")
    p.add_argument("--lane_width", type=float, default=DEFAULT_LANE_WIDTH_M)
    p.add_argument("--line_width_tag", type=float, default=DEFAULT_LINE_WIDTH_TAG_M)
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    conv = KCityV5Converter(args.input_dir, args.output_osm, lane_width_m=args.lane_width, line_width_tag_m=args.line_width_tag)
    conv.run()


