import argparse
import os
from typing import Dict, List, Optional, Tuple

import geopandas as gpd
import numpy as np
from lxml import etree
from shapely.geometry import LineString, Point, Polygon, MultiLineString, GeometryCollection
from shapely.ops import linemerge


# -------------------- Minimal helpers --------------------

COORD_ROUND_DECIMALS = 7


def _round_coord(lon: float, lat: float) -> Tuple[float, float]:
    return (round(lon, COORD_ROUND_DECIMALS), round(lat, COORD_ROUND_DECIMALS))


def _coords_2d(geom) -> List[Tuple[float, float]]:
    if isinstance(geom, LineString):
        return [(c[0], c[1]) for c in geom.coords]
    if isinstance(geom, MultiLineString):
        pts: List[Tuple[float, float]] = []
        for g in geom.geoms:
            pts.extend(_coords_2d(g))
        return pts
    if isinstance(geom, GeometryCollection):
        pts: List[Tuple[float, float]] = []
        for g in geom.geoms:
            pts.extend(_coords_2d(g))
        return pts
    return []


def _ordered_coords_from_geom(geom) -> List[Tuple[float, float]]:
    # Build an ordered polyline by connecting segment endpoints geometrically
    # Supports LineString, MultiLineString, GeometryCollection of LineStrings
    # 1) Collect segments
    segments: List[List[Tuple[float, float]]] = []
    def collect(g):
        if isinstance(g, LineString):
            coords = [(c[0], c[1]) for c in g.coords]
            if len(coords) >= 2:
                segments.append(coords)
        elif isinstance(g, MultiLineString) or isinstance(g, GeometryCollection):
            for sub in g.geoms:
                collect(sub)
    collect(geom)
    if not segments:
        return _coords_2d(geom)
    # 2) Greedy connect by nearest endpoints; reverse segment if needed
    used = [False] * len(segments)
    # pick first non-empty
    chain: List[Tuple[float, float]] = list(segments[0])
    used[0] = True
    def dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        return (dx * dx + dy * dy) ** 0.5
    while not all(used):
        best_i = -1
        best_mode = None  # 'head' or 'tail', 'normal' or 'reverse'
        best_d = 1e18
        head = chain[0]
        tail = chain[-1]
        for i, seg in enumerate(segments):
            if used[i]:
                continue
            s = seg[0]
            e = seg[-1]
            # connect tail -> s (normal)
            d1 = dist(tail, s)
            if d1 < best_d:
                best_d = d1
                best_i = i
                best_mode = ('tail', 'normal')
            # tail -> e (reverse)
            d2 = dist(tail, e)
            if d2 < best_d:
                best_d = d2
                best_i = i
                best_mode = ('tail', 'reverse')
            # s -> head (normal prepend)
            d3 = dist(e, head)
            if d3 < best_d:
                best_d = d3
                best_i = i
                best_mode = ('head', 'normal')
            # e -> head (reverse prepend)
            d4 = dist(s, head)
            if d4 < best_d:
                best_d = d4
                best_i = i
                best_mode = ('head', 'reverse')
        if best_i < 0:
            break
        seg = segments[best_i]
        used[best_i] = True
        mode_end, mode_dir = best_mode
        seg_coords = list(seg)
        if mode_dir == 'reverse':
            seg_coords.reverse()
        if mode_end == 'tail':
            # avoid duplicate vertex
            to_add = seg_coords[1:] if chain[-1] == seg_coords[0] else seg_coords
            chain.extend(to_add)
        else:  # prepend to head
            to_add = seg_coords[:-1] if chain[0] == seg_coords[-1] else seg_coords
            chain = to_add + chain
    return chain


def _ensure_polyline_coords(geom) -> List[Tuple[float, float]]:
    # Try ordered reconstruction first
    coords = _ordered_coords_from_geom(geom)
    if len(coords) >= 2:
        return coords
    # Fallback to simple coords extraction
    coords = _coords_2d(geom)
    if len(coords) >= 2:
        return coords
    # As a last resort, try to merge
    try:
        merged = linemerge(geom)
        coords = _coords_2d(merged)
        if len(coords) >= 2:
            return coords
    except Exception:
        pass
    return coords


def _norm_link_value(v: object) -> str:
    if v is None:
        return ""
    try:
        import math
        if isinstance(v, float) and math.isnan(v):
            return ""
    except Exception:
        pass
    try:
        f = float(v)
        i = int(round(f))
        if abs(f - i) < 1e-9:
            return str(i)
        return str(f)
    except Exception:
        s = str(v).strip()
        try:
            f2 = float(s)
            i2 = int(round(f2))
            if abs(f2 - i2) < 1e-9:
                return str(i2)
            return str(f2)
        except Exception:
            return s


# -------------------- Simple OSM builder --------------------

class IdGen:
    def __init__(self) -> None:
        self.node = 1
        self.way = 1
        self.rel = 1

    def next_node(self) -> str:
        nid = str(self.node)
        self.node += 1
        return nid

    def next_way(self) -> str:
        wid = str(self.way)
        self.way += 1
        return wid

    def next_rel(self) -> str:
        rid = str(self.rel)
        self.rel += 1
        return rid


class Osm:
    def __init__(self) -> None:
        self.ids = IdGen()
        self.nodes: Dict[str, Tuple[float, float]] = {}
        self.node_map: Dict[Tuple[float, float], str] = {}
        self.ways: Dict[str, Dict] = {}
        self.rels: Dict[str, Dict] = {}

    def add_node(self, lon: float, lat: float) -> str:
        coord = _round_coord(lon, lat)
        if coord in self.node_map:
            return self.node_map[coord]
        nid = self.ids.next_node()
        self.nodes[nid] = coord
        self.node_map[coord] = nid
        return nid

    def add_way(self, coords: List[Tuple[float, float]], tags: Optional[Dict[str, str]] = None) -> Optional[str]:
        if not coords:
            return None
        dedup: List[Tuple[float, float]] = [coords[0]]
        for x, y in coords[1:]:
            r = _round_coord(x, y)
            if r != _round_coord(*dedup[-1]):
                dedup.append((x, y))
        if len(dedup) < 2:
            return None
        nrefs = [self.add_node(x, y) for x, y in dedup]
        wid = self.ids.next_way()
        self.ways[wid] = {"nodes": nrefs, "tags": tags or {}}
        return wid

    def add_rel_fixed_id(self, rid: str, members: List[Tuple[str, str, str]], tags: Optional[Dict[str, str]] = None) -> str:
        self.rels[str(rid)] = {"members": members, "tags": tags or {}}
        return str(rid)

    def save(self, path: str) -> None:
        root = etree.Element("osm", version="0.6", generator="KCityV5Strict")
        # Lanelet2 meta tags
        etree.SubElement(root, "tag", k="format_version", v="1.1")
        etree.SubElement(root, "tag", k="map_version", v="1")
        for nid, (lon, lat) in self.nodes.items():
            etree.SubElement(root, "node", id=nid, lat=str(lat), lon=str(lon))
        for wid, w in self.ways.items():
            we = etree.SubElement(root, "way", id=wid)
            for nref in w["nodes"]:
                etree.SubElement(we, "nd", ref=nref)
            for k, v in w["tags"].items():
                etree.SubElement(we, "tag", k=str(k), v=str(v))
        for rid, r in self.rels.items():
            re = etree.SubElement(root, "relation", id=rid)
            for t, ref, role in r["members"]:
                etree.SubElement(re, "member", type=t, ref=str(ref), role=role)
            for k, v in r["tags"].items():
                etree.SubElement(re, "tag", k=str(k), v=str(v))
        with open(path, "wb") as f:
            f.write(etree.tostring(root, pretty_print=True, xml_declaration=True, encoding="UTF-8"))


# -------------------- Strict converter --------------------

def load_gdf(input_dir: str, name: str) -> Optional[gpd.GeoDataFrame]:
    p = os.path.join(input_dir, f"{name}.shp")
    if not os.path.exists(p):
        return None
    gdf = gpd.read_file(p)
    if gdf.crs is not None and gdf.crs.to_epsg() != 4326:
        gdf = gdf.to_crs("EPSG:4326")
    gdf = gdf[~gdf.geometry.is_empty & gdf.geometry.is_valid]
    return gdf


def build_lanelets_strict(input_dir: str, output_osm: str) -> None:
    g_path = load_gdf(input_dir, "path")
    g_lr = load_gdf(input_dir, "LR_for_path")
    g_stop = load_gdf(input_dir, "stopline")
    g_side = load_gdf(input_dir, "sidewalk")

    osm = Osm()

    # Maps for LR attributes
    left_map: Dict[str, List[int]] = {}
    right_map: Dict[str, List[int]] = {}
    if g_lr is not None and not g_lr.empty:
        if "LeftLink" in g_lr.columns:
            for i, v in enumerate(g_lr["LeftLink"]):
                s = _norm_link_value(v)
                if s:
                    left_map.setdefault(s, []).append(i)
        if "RightLink" in g_lr.columns:
            for i, v in enumerate(g_lr["RightLink"]):
                s = _norm_link_value(v)
                if s:
                    right_map.setdefault(s, []).append(i)

    # Lanelets (strict): for each path row, use Link and pick exactly matching LeftLink/RightLink
    if g_path is not None and not g_path.empty and g_lr is not None and not g_lr.empty:
        for _, row in g_path.iterrows():
            link = _norm_link_value(row.get("Link", row.get("ID", "")))
            if not link:
                continue
            geom = row.geometry
            if not isinstance(geom, (LineString, MultiLineString)):
                continue
            if isinstance(geom, MultiLineString):
                merged = linemerge(geom)
                geom = merged if isinstance(merged, LineString) else list(merged.geoms)[0]
            # flatten to 2D if needed
            try:
                if hasattr(geom, "has_z") and geom.has_z:
                    geom = LineString([(c[0], c[1]) for c in geom.coords])
            except Exception:
                pass

            l_idx = left_map.get(link, [])
            r_idx = right_map.get(link, [])
            if not l_idx or not r_idx:
                continue  # strict: require both

            # choose first geometry deterministically (no heuristics)
            l_geom = g_lr.geometry.iloc[l_idx[0]]
            r_geom = g_lr.geometry.iloc[r_idx[0]]
            # 2D
            try:
                if hasattr(l_geom, "has_z") and l_geom.has_z:
                    l_geom = LineString([(c[0], c[1]) for c in l_geom.coords])
                if hasattr(r_geom, "has_z") and r_geom.has_z:
                    r_geom = LineString([(c[0], c[1]) for c in r_geom.coords])
            except Exception:
                pass

            # build ordered coordinates by connecting segment endpoints
            left_coords = _ensure_polyline_coords(l_geom)
            right_coords = _ensure_polyline_coords(r_geom)
            center_coords = _ensure_polyline_coords(geom)

            left_w = osm.add_way(left_coords, {"type": "line_thin", "subtype": "solid"})
            right_w = osm.add_way(right_coords, {"type": "line_thin", "subtype": "solid"})
            center_w = osm.add_way(center_coords, {})
            if not left_w or not right_w or not center_w:
                continue
            rel_tags = {"type": "lanelet", "subtype": "road", "id": link}
            osm.add_rel_fixed_id(link, [("way", left_w, "left"), ("way", right_w, "right"), ("way", center_w, "centerline")], rel_tags)

    # Stop lines (as-is)
    if g_stop is not None and not g_stop.empty:
        for geom in g_stop.geometry:
            if isinstance(geom, LineString):
                try:
                    if hasattr(geom, "has_z") and geom.has_z:
                        geom = LineString([(c[0], c[1]) for c in geom.coords])
                except Exception:
                    pass
                osm.add_way(_coords_2d(geom), {"type": "stop_line"})

    # Crosswalks: accept both polygon and pair of lines in LR_for_path (Kind, Type may carry clues)
    # 1) From sidewalk polygons (deterministic min-rect)
    if g_side is not None and not g_side.empty:
        for poly in g_side.geometry:
            if not isinstance(poly, Polygon):
                continue
            rect = poly.minimum_rotated_rectangle
            if not isinstance(rect, Polygon):
                continue
            coords = list(rect.exterior.coords)[:-1]
            if len(coords) != 4:
                continue
            edges = [LineString([coords[i], coords[(i + 1) % 4]]) for i in range(4)]
            lens = [edges[i].length for i in range(4)]
            order = np.argsort(lens)
            e1, e2 = edges[order[-1]], edges[order[-2]]
            lw = osm.add_way(_ordered_coords_from_geom(e1), {"type": "line_thin", "subtype": "solid"})
            rw = osm.add_way(_ordered_coords_from_geom(e2), {"type": "line_thin", "subtype": "solid"})
            if lw and rw:
                tags = {"type": "lanelet", "subtype": "crosswalk", "participant:pedestrian": "yes"}
                # numeric, non-colliding relation id for crosswalks
                rid_num = 100000 + int(osm.ids.next_rel())
                osm.rels[str(rid_num)] = {"members": [("way", lw, "left"), ("way", rw, "right")], "tags": tags}

    # 2) From LR_for_path lines explicitly marked as crosswalk (if present via Type/Kind)
    if g_lr is not None and not g_lr.empty and 'Type' in g_lr.columns:
        # Heuristic: group lines having same LeftLink/RightLink but special crosswalk type
        # User can encode crosswalk via Type==211/Kind==530 like stoplines; adjust if needed
        cw_lines: List[Tuple[str, LineString]] = []
        for _, r in g_lr.iterrows():
            t = str(r.get('Type',''))
            k = str(r.get('Kind',''))
            if t in {'213','214','215'} or k in {'531','532','533'}:  # flexible list, user can refine
                cw_lines.append(( _norm_link_value(r.get('LeftLink','')) + '|' + _norm_link_value(r.get('RightLink','')) , r.geometry))
        # pair by key
        used = set()
        for i in range(len(cw_lines)):
            if i in used:
                continue
            key_i, gi = cw_lines[i]
            for j in range(i+1, len(cw_lines)):
                if j in used:
                    continue
                key_j, gj = cw_lines[j]
                if key_i == key_j:
                    lw = osm.add_way(_ordered_coords_from_geom(gi), {"type": "line_thin", "subtype": "solid"})
                    rw = osm.add_way(_ordered_coords_from_geom(gj), {"type": "line_thin", "subtype": "solid"})
                    if lw and rw:
                        tags = {"type": "lanelet", "subtype": "crosswalk", "participant:pedestrian": "yes"}
                        rid_num = 100000 + int(osm.ids.next_rel())
                        osm.rels[str(rid_num)] = {"members": [("way", lw, "left"), ("way", rw, "right")], "tags": tags}
                        used.add(i); used.add(j); break

    osm.save(output_osm)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Strict converter: path.Link with LR_for_path.LeftLink/RightLink to lanelet; plus stoplines and crosswalks only")
    p.add_argument("--input_dir", required=True)
    p.add_argument("--output_osm", required=True)
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    build_lanelets_strict(args.input_dir, args.output_osm)


