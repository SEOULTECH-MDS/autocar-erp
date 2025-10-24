import os
import argparse
from typing import Dict, List, Tuple
import geopandas as gpd
import numpy as np
from shapely.geometry import LineString, MultiLineString, GeometryCollection
from shapely.ops import linemerge, transform as shp_transform
from pyproj import Transformer
from lxml import etree


def _to_linestring(geom) -> LineString:
    if geom is None or geom.is_empty:
        return LineString()
    if isinstance(geom, LineString):
        return geom
    if isinstance(geom, MultiLineString):
        try:
            merged = linemerge(geom)
        except Exception:
            merged = geom
        if isinstance(merged, LineString):
            return merged
        if isinstance(merged, MultiLineString):
            # pick the longest
            parts = list(merged.geoms)
            if not parts:
                return LineString()
            parts.sort(key=lambda g: g.length, reverse=True)
            return parts[0]
    if isinstance(geom, GeometryCollection):
        # take longest linestring among parts
        lines = [g for g in geom.geoms if isinstance(g, (LineString, MultiLineString))]
        if not lines:
            return LineString()
        best = None
        best_len = -1.0
        for g in lines:
            ls = _to_linestring(g)
            if ls.length > best_len:
                best = ls
                best_len = ls.length
        return best if best is not None else LineString()
    return LineString()


def _clean_linestring(geom) -> LineString:
    ls = _to_linestring(geom)
    if ls.is_empty:
        return LineString()
    coords = list(ls.coords)
    dedup = []
    for c in coords:
        if not dedup or (c[0] != dedup[-1][0] or c[1] != dedup[-1][1]):
            dedup.append(c)
    if len(dedup) < 2:
        return LineString()
    return LineString(dedup)


def _resample(ls: LineString, step: float) -> LineString:
    if ls.length == 0 or step <= 0:
        return ls
    n = max(2, int(np.ceil(ls.length / step)) + 1)
    dists = np.linspace(0, ls.length, n)
    pts = [ls.interpolate(d) for d in dists]
    return LineString([(p.x, p.y) for p in pts])


class OsmIds:
    def __init__(self) -> None:
        self.next_node = 1
        self.next_way = 1
        self.next_rel = 1

    def n(self) -> int:
        i = self.next_node; self.next_node += 1; return i

    def w(self) -> int:
        i = self.next_way; self.next_way += 1; return i

    def r(self) -> int:
        i = self.next_rel; self.next_rel += 1; return i


class Osm:
    def __init__(self) -> None:
        self.ids = OsmIds()
        self.nodes: Dict[Tuple[float, float], int] = {}
        self.ways: Dict[int, Dict] = {}
        self.rels: Dict[int, Dict] = {}

    def node(self, lon: float, lat: float) -> int:
        key = (round(lon, 7), round(lat, 7))
        if key in self.nodes:
            return self.nodes[key]
        nid = self.ids.n()
        self.nodes[key] = nid
        return nid

    def way(self, coords: List[Tuple[float, float]], tags: Dict[str, str]) -> int:
        if len(coords) < 2:
            return -1
        w_id = self.ids.w()
        nds = [self.node(x, y) for x, y in coords]
        self.ways[w_id] = {'nds': nds, 'tags': dict(tags)}
        return w_id

    def rel(self, members: List[Tuple[str, int, str]], tags: Dict[str, str]) -> int:
        r_id = self.ids.r()
        self.rels[r_id] = {'members': list(members), 'tags': dict(tags)}
        return r_id

    def rel_fixed_id(self, fixed_id: int, members: List[Tuple[str, int, str]], tags: Dict[str, str]) -> int:
        rid = int(fixed_id)
        # ensure uniqueness: if exists, overwrite
        self.rels[rid] = {'members': list(members), 'tags': dict(tags)}
        return rid

    def to_etree(self) -> etree._Element:
        root = etree.Element('osm', version='0.6', generator='boundary_to_lanelet')
        etree.SubElement(root, 'tag', k='format_version', v='1.1')
        etree.SubElement(root, 'tag', k='map_version', v='1.0')
        for (lon, lat), nid in self.nodes.items():
            etree.SubElement(root, 'node', id=str(nid), lon=str(lon), lat=str(lat))
        for wid, w in self.ways.items():
            w_el = etree.SubElement(root, 'way', id=str(wid))
            for nid in w['nds']:
                etree.SubElement(w_el, 'nd', ref=str(nid))
            for k, v in w['tags'].items():
                etree.SubElement(w_el, 'tag', k=str(k), v=str(v))
        for rid, r in self.rels.items():
            r_el = etree.SubElement(root, 'relation', id=str(rid))
            for t, ref, role in r['members']:
                etree.SubElement(r_el, 'member', type=t, ref=str(ref), role=str(role))
            for k, v in r['tags'].items():
                etree.SubElement(r_el, 'tag', k=str(k), v=str(v))
        return root

    def save(self, path: str) -> None:
        root = self.to_etree()
        xml = etree.tostring(root, pretty_print=True, xml_declaration=True, encoding='UTF-8')
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'wb') as f:
            f.write(xml)


def compute_centerline(left: LineString, right: LineString, sample_step_m: float = 1.0) -> LineString:
    # Work in metric CRS for uniform sampling, then transform back
    fwd = Transformer.from_crs('EPSG:4326', 'EPSG:3857', always_xy=True)
    inv = Transformer.from_crs('EPSG:3857', 'EPSG:4326', always_xy=True)

    left_m = shp_transform(lambda x, y: fwd.transform(x, y), left)
    right_m = shp_transform(lambda x, y: fwd.transform(x, y), right)
    if left_m.length == 0 or right_m.length == 0:
        return LineString()

    n = max(2, int(np.ceil(max(left_m.length, right_m.length) / sample_step_m)) + 1)
    pts = []
    for i in range(n):
        t = i / (n - 1)
        dl = t * left_m.length
        dr = t * right_m.length
        pl = left_m.interpolate(dl)
        pr = right_m.interpolate(dr)
        cx = (pl.x + pr.x) / 2.0
        cy = (pl.y + pr.y) / 2.0
        lon, lat = inv.transform(cx, cy)
        pts.append((lon, lat))
    return _clean_linestring(LineString(pts))


def build_lanelets(boundary_shp: str, output_osm: str) -> None:
    gdf = gpd.read_file(boundary_shp)
    if gdf.crs is None or gdf.crs.to_epsg() != 4326:
        gdf = gdf.to_crs('EPSG:4326')

    # Normalize columns
    cols = {c.lower(): c for c in gdf.columns}
    left_col = cols.get('left_line') or cols.get('left') or cols.get('leftid')
    right_col = cols.get('right_line') or cols.get('right') or cols.get('rightid')
    type_col = cols.get('type')
    subtype_col = cols.get('subtype')

    # Group boundaries by lanelet id, collecting left/right lines
    lanelet_map: Dict[int, Dict[str, List[LineString]]] = {}
    for _, row in gdf.iterrows():
        geom: LineString = _clean_linestring(row.geometry)
        if geom.is_empty:
            continue
        if type_col and str(row[type_col]) not in ('line_thin', 'line_thick', 'solid', 'dashed'):
            # still accept; boundary lines expected mostly
            pass
        left_id = row[left_col] if left_col else None
        right_id = row[right_col] if right_col else None
        # Decide lanelet id and side
        if left_id is not None and not np.isnan(left_id):
            lid = int(left_id)
            lanelet_map.setdefault(lid, {'left': [], 'right': []})
            lanelet_map[lid]['left'].append(geom)
        if right_id is not None and not np.isnan(right_id):
            rid = int(right_id)
            lanelet_map.setdefault(rid, {'left': [], 'right': []})
            lanelet_map[rid]['right'].append(geom)

    # Merge multi-segment sides and build OSM
    osm = Osm()
    for lid, sides in lanelet_map.items():
        left_geom = linemerge([ls for ls in sides['left']]) if sides['left'] else LineString()
        right_geom = linemerge([ls for ls in sides['right']]) if sides['right'] else LineString()
        left_geom = _clean_linestring(left_geom)
        right_geom = _clean_linestring(right_geom)
        if left_geom.is_empty or right_geom.is_empty:
            # skip incomplete lanelet
            continue
        # Ensure same direction roughly: if end-points are reversed, flip right to match left
        def _maybe_flip(to_fix: LineString, ref: LineString) -> LineString:
            if to_fix.is_empty or ref.is_empty:
                return to_fix
            ref0 = np.array(ref.coords[0])
            to0 = np.array(to_fix.coords[0])
            toE = np.array(to_fix.coords[-1])
            d_same_len = np.linalg.norm(ref0 - to0)
            d_flip_len = np.linalg.norm(ref0 - toE)
            return to_fix if d_same_len <= d_flip_len else LineString(list(to_fix.coords)[::-1])

        right_geom = _maybe_flip(right_geom, left_geom)

        center = compute_centerline(left_geom, right_geom)
        if center.is_empty:
            continue

        left_w = osm.way(list(left_geom.coords), {'type': 'line_thin', 'subtype': 'solid'})
        right_w = osm.way(list(right_geom.coords), {'type': 'line_thin', 'subtype': 'solid'})
        center_w = osm.way(list(center.coords), {})
        if left_w < 0 or right_w < 0 or center_w < 0:
            continue

        rel_tags = {'type': 'lanelet', 'subtype': 'road', 'id': str(lid)}
        osm.rel_fixed_id(lid, [('way', left_w, 'left'), ('way', right_w, 'right'), ('way', center_w, 'centerline')], rel_tags)

    osm.save(output_osm)


def main():
    p = argparse.ArgumentParser(description='Create Lanelet2 OSM from boundary shapefile with left/right ids')
    p.add_argument('--boundary-shp', required=True)
    p.add_argument('--output-osm', required=True)
    p.add_argument('--sample-step', type=float, default=0.8, help='centerline sampling step (deg metric approx)')
    args = p.parse_args()

    build_lanelets(args.boundary_shp, args.output_osm)


if __name__ == '__main__':
    main()


