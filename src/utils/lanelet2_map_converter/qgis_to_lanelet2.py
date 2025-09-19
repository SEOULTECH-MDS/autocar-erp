import os
import sys
import argparse
import math
from typing import Dict, List, Tuple, Optional, Iterable

import geopandas as gpd
from shapely.geometry import LineString, MultiLineString, Polygon, MultiPolygon
from shapely.ops import linemerge
from shapely.ops import unary_union
from shapely.geometry import GeometryCollection
from lxml import etree

# Notes:
# - We reproject input layers to EPSG:3857 for metric geometry operations (offset, buffer, clip),
#   then transform back to EPSG:4326 (lon/lat) for OSM output.
# - Required layers by default: path.shp, dashed_line.shp, LR_virtual.shp, left_right_lines.shp
# - Attributes used: is_path (1 means target), left_lane, right_lane (numbers/lane ids)
# - Rule: For lines where is_path==1 and left_lane==right_lane, create a lanelet with that id.
#   Use path geometry as centerline, clipped to the ROI of the lane id (union of selected lines buffered).
#   Non-path lines (is_path != 1) are emitted as line strings with visualization tags (dashed/virtual/solid).

DEFAULT_LANE_WIDTH_M = 3.5
LEFT_RIGHT_BUFFER_M = 1.0
CENTER_CLIP_BUFFER_M = 1.5


def load_layer(path_dir: str, basename: str) -> Optional[gpd.GeoDataFrame]:
    # Case-insensitive search for files named like basename.(shp|geojson|gpkg)
    target = basename.lower()
    found_path = None
    try:
        for fname in os.listdir(path_dir):
            name, ext = os.path.splitext(fname)
            if ext.lower() not in ('.shp', '.geojson', '.gpkg'):
                continue
            if name.lower() == target:
                found_path = os.path.join(path_dir, fname)
                break
    except Exception:
        pass

    # Fallback to direct joins if listing fails
    if not found_path:
        for ext in ('.shp', '.geojson', '.gpkg'):
            candidate = os.path.join(path_dir, basename + ext)
            if os.path.exists(candidate):
                found_path = candidate
                break

    if not found_path:
        print(f"Layer {basename} not found under {path_dir}")
        return None

    try:
        gdf = gpd.read_file(found_path)
        # Ensure CRS
        if gdf.crs is None:
            # Assume WGS84 if not provided
            gdf.set_crs(epsg=4326, inplace=True)
        # Reproject to metric
        if gdf.crs.to_epsg() != 3857:
            gdf = gdf.to_crs(epsg=3857)
        # Normalize columns to lower case
        gdf.columns = [str(c).lower() for c in gdf.columns]
        return gdf
    except Exception as e:
        print(f"Failed to load {found_path}: {e}")
        return None


class OsmBuilder:
    def __init__(self):
        self.nodes: Dict[str, Tuple[float, float]] = {}
        self.node_coord_index: Dict[Tuple[float, float], str] = {}
        self.ways: Dict[str, Dict] = {}
        self.relations: Dict[str, Dict] = {}
        self.node_id = 1
        self.way_id = 1
        self.rel_id = 1
        self.used_relation_ids = set()

    def add_node(self, lon: float, lat: float) -> str:
        # Round to avoid duplicates from tiny float noise
        key = (round(lon, 7), round(lat, 7))
        if key in self.node_coord_index:
            return self.node_coord_index[key]
        nid = str(self.node_id)
        self.node_id += 1
        self.nodes[nid] = key
        self.node_coord_index[key] = nid
        return nid

    def add_linestring_way(self, line: LineString, tags: Dict[str, str]) -> Optional[str]:
        if line is None or line.is_empty:
            return None
        # Ensure simple LineString
        if not isinstance(line, LineString):
            return None
        # Build nodes
        node_ids: List[str] = []
        for coord in list(line.coords):
            # coord can be (x,y) or (x,y,z)
            x = float(coord[0])
            y = float(coord[1])
            nid = self.add_node(x, y)
            if not node_ids or node_ids[-1] != nid:
                node_ids.append(nid)
        if len(node_ids) < 2:
            return None
        wid = str(self.way_id)
        self.way_id += 1
        self.ways[wid] = {
            'nodes': node_ids,
            'tags': tags.copy() if tags else {}
        }
        return wid

    def add_lanelet_relation(self, left_way: str, right_way: str, center_way: str, lane_id: str, prefer_relation_id: Optional[str] = None):
        # Try to use provided lane_id as relation id if possible
        rid = None
        if prefer_relation_id is not None:
            candidate = str(prefer_relation_id)
            if candidate not in self.relations:
                rid = candidate
        if rid is None:
            rid = str(self.rel_id)
            self.rel_id += 1
        self.used_relation_ids.add(rid)
        self.relations[rid] = {
            'members': [
                ('way', left_way, 'left'),
                ('way', right_way, 'right'),
                ('way', center_way, 'centerline')
            ],
            'tags': {
                'type': 'lanelet',
                'subtype': 'road',
                'id': str(lane_id)
            }
        }

    def to_etree(self) -> etree._Element:
        root = etree.Element('osm', version='0.6', generator='qgis_to_lanelet2')
        for nid, (lon, lat) in self.nodes.items():
            etree.SubElement(root, 'node', id=nid, lat=str(lat), lon=str(lon))
        for wid, w in self.ways.items():
            w_elem = etree.SubElement(root, 'way', id=wid)
            for nd in w['nodes']:
                etree.SubElement(w_elem, 'nd', ref=nd)
            for k, v in (w.get('tags') or {}).items():
                etree.SubElement(w_elem, 'tag', k=str(k), v=str(v))
        for rid, r in self.relations.items():
            r_elem = etree.SubElement(root, 'relation', id=rid)
            for t, ref, role in r['members']:
                etree.SubElement(r_elem, 'member', type=t, ref=ref, role=role)
            for k, v in (r.get('tags') or {}).items():
                etree.SubElement(r_elem, 'tag', k=str(k), v=str(v))
        return root

    def save(self, output_file: str):
        root = self.to_etree()
        xml_bytes = etree.tostring(root, pretty_print=True, xml_declaration=True, encoding='UTF-8')
        with open(output_file, 'wb') as f:
            f.write(xml_bytes)
        print(f"Saved OSM: {output_file}")


def lines_to_multiline(lines: Iterable[LineString]) -> Optional[MultiLineString]:
    lines = [ln for ln in lines if isinstance(ln, LineString) and not ln.is_empty]
    if not lines:
        return None
    merged = linemerge(lines)
    if isinstance(merged, LineString):
        return MultiLineString([merged])
    if isinstance(merged, MultiLineString):
        return merged
    return None


def safe_geometry(geom) -> Optional[LineString]:
    if geom is None or geom.is_empty:
        return None
    if isinstance(geom, LineString):
        return geom
    return None


def main():
    parser = argparse.ArgumentParser(description='Convert QGIS-exported layers to Lanelet2 OSM.')
    parser.add_argument('--input-dir', required=True, help='Directory containing layers (path, dashed_line, LR_virtual, left_right_lines)')
    parser.add_argument('--output-osm', required=True, help='Output OSM filepath')
    parser.add_argument('--lane-width', type=float, default=DEFAULT_LANE_WIDTH_M)
    args = parser.parse_args()

    path_gdf = load_layer(args.input_dir, 'path')
    dashed_gdf = load_layer(args.input_dir, 'dashed_line')
    virtual_gdf = load_layer(args.input_dir, 'lr_virtual')
    lr_gdf = load_layer(args.input_dir, 'left_right_lines')

    if (lr_gdf is None) and (dashed_gdf is None) and (virtual_gdf is None):
        print('Error: No side line layers found (left_right_lines, dashed_line, LR_virtual).')
        sys.exit(1)

    # Helper to resolve column name robustly
    def col(gdf, name):
        name = name.lower()
        if name in gdf.columns:
            return name
        # try common variants
        for c in gdf.columns:
            if c.replace('_', '').lower() == name.replace('_', '').lower():
                return c
        return None

    # Collect side lines by lane id across all layers
    def collect_side(gdf: Optional[gpd.GeoDataFrame], side: str, subtype: str) -> Dict[str, List[LineString]]:
        res: Dict[str, List[LineString]] = {}
        if gdf is None or gdf.empty:
            return res
        is_path_c = col(gdf, 'is_path')
        left_c = col(gdf, 'left_lane')
        right_c = col(gdf, 'right_lane')
        if is_path_c is None or left_c is None or right_c is None:
            return res
        df = gdf[gdf[is_path_c].astype(float) == 1.0]
        key_c = left_c if side == 'left' else right_c
        for _, row in df.iterrows():
            lane_id = str(row.get(key_c))
            geom = row.geometry
            if isinstance(geom, LineString) and not geom.is_empty:
                res.setdefault(lane_id, []).append(geom)
            elif isinstance(geom, MultiLineString):
                for ln in geom.geoms:
                    if isinstance(ln, LineString) and not ln.is_empty:
                        res.setdefault(lane_id, []).append(ln)
        return res

    left_solid = collect_side(lr_gdf, 'left', 'solid')
    right_solid = collect_side(lr_gdf, 'right', 'solid')
    left_dashed = collect_side(dashed_gdf, 'left', 'dashed')
    right_dashed = collect_side(dashed_gdf, 'right', 'dashed')
    left_virtual = collect_side(virtual_gdf, 'left', 'virtual')
    right_virtual = collect_side(virtual_gdf, 'right', 'virtual')

    all_left_ids = set(left_solid.keys()) | set(left_dashed.keys()) | set(left_virtual.keys())
    all_right_ids = set(right_solid.keys()) | set(right_dashed.keys()) | set(right_virtual.keys())
    lane_ids = sorted(all_left_ids & all_right_ids)

    # Path is optional now; we will derive centerline from sides directly
    path_ml = None

    # Prepare OSM builder; all export must be in EPSG:4326
    builder = OsmBuilder()

    # Helper to project back to 4326
    to_ll = lambda gdf: gdf.to_crs(epsg=4326)

    # Utility: unify multiple parts into a single LineString (take longest)
    def _reverse_ls(ls: LineString) -> LineString:
        return LineString(list(ls.coords)[::-1])

    def _pt(p):
        return (float(p.x), float(p.y))

    def _dist_xy(a: Tuple[float,float], b: Tuple[float,float]) -> float:
        dx = a[0]-b[0]; dy = a[1]-b[1]
        return (dx*dx+dy*dy)**0.5

    def _endpoints(ls: LineString) -> Tuple[Tuple[float,float], Tuple[float,float]]:
        coords = list(ls.coords)
        return (float(coords[0][0]), float(coords[0][1])), (float(coords[-1][0]), float(coords[-1][1]))

    def _order_lines_to_single(parts: List[LineString], snap_tol: float = 1.0) -> Optional[LineString]:
        parts = [ln for ln in parts if isinstance(ln, LineString) and not ln.is_empty]
        if not parts:
            return None
        used = [False]*len(parts)
        # start with the longest
        start_idx = max(range(len(parts)), key=lambda i: parts[i].length)
        used[start_idx] = True
        chain_coords = list(parts[start_idx].coords)
        # Greedily append/prepend nearest segment if within tolerance
        changed = True
        while changed:
            changed = False
            head = (chain_coords[0][0], chain_coords[0][1])
            tail = (chain_coords[-1][0], chain_coords[-1][1])
            best_i = -1; best_side = None; best_rev = False; best_d = 1e9
            for i, ln in enumerate(parts):
                if used[i]:
                    continue
                s, e = _endpoints(ln)
                for side in ('append','prepend'):
                    ref = tail if side=='append' else head
                    for rev in (False, True):
                        a = e if not rev else s
                        d = _dist_xy(ref, a)
                        if d < best_d:
                            best_d = d; best_i = i; best_side = side; best_rev = rev
            if best_i >= 0 and best_d <= snap_tol:
                ln = parts[best_i]
                if best_rev:
                    ln = _reverse_ls(ln)
                if best_side == 'append':
                    chain_coords.extend(list(ln.coords)[1:])
                else:
                    chain_coords = list(ln.coords)[:-1] + chain_coords
                used[best_i] = True
                changed = True
        try:
            ls = LineString(chain_coords)
            return ls if not ls.is_empty and len(ls.coords) >= 2 else None
        except Exception:
            return None

    def unify_lines(lines: List[LineString]) -> Optional[LineString]:
        lines = [ln for ln in (lines or []) if isinstance(ln, LineString) and not ln.is_empty]
        if not lines:
            return None
        merged = linemerge(lines)
        if isinstance(merged, LineString):
            return merged
        if isinstance(merged, MultiLineString):
            # Try greedy ordering with small snap tolerance (meters)
            ordered = _order_lines_to_single(list(merged.geoms), snap_tol=0.5)
            if ordered is not None:
                return ordered
            return max(merged.geoms, key=lambda ln: ln.length) if merged.geoms else None
        return None

    # Choose side geometry with priority: solid > dashed > virtual
    def choose_side_geom(lane_id: str, side: str) -> Tuple[Optional[LineString], Optional[str]]:
        if side == 'left':
            for source, tag in ((left_solid, 'solid'), (left_dashed, 'dashed'), (left_virtual, 'virtual')):
                if lane_id in source:
                    return unify_lines(source[lane_id]), tag
        else:
            for source, tag in ((right_solid, 'solid'), (right_dashed, 'dashed'), (right_virtual, 'virtual')):
                if lane_id in source:
                    return unify_lines(source[lane_id]), tag
        return None, None

    # Build centerline from left/right by sampling midpoints
    def centerline_from_sides(left_ls: LineString, right_ls: LineString, samples: int = 200) -> Optional[LineString]:
        if not isinstance(left_ls, LineString) or not isinstance(right_ls, LineString):
            return None
        if left_ls.is_empty or right_ls.is_empty:
            return None
        # Ensure both sides flow in the same direction
        ls0s, ls0e = _endpoints(left_ls)
        rs0s, rs0e = _endpoints(right_ls)
        if _dist_xy(ls0s, rs0e) < _dist_xy(ls0s, rs0s):
            right_ls = _reverse_ls(right_ls)
        # Resample both sides to comparable lengths by normalizing to [0,1] along length
        # Then compute midpoints. If either side is shorter, clamp at end.
        pts = []
        max_sep = max(args.lane_width * 3.0, 6.0)
        run = []
        best_run = []
        for i in range(samples+1):
            t = i / samples
            try:
                pl = left_ls.interpolate(t, normalized=True)
            except Exception:
                pl = left_ls.interpolate(left_ls.length * t)
            try:
                pr = right_ls.interpolate(t, normalized=True)
            except Exception:
                pr = right_ls.interpolate(right_ls.length * t)
            mx = (pl.x + pr.x) / 2.0
            my = (pl.y + pr.y) / 2.0
            sep = _dist_xy(_pt(pl), _pt(pr))
            if sep <= max_sep:
                if not run or (run[-1][0] != mx or run[-1][1] != my):
                    run.append((mx, my))
            else:
                if len(run) > len(best_run):
                    best_run = run
                run = []
        if len(run) > len(best_run):
            best_run = run
        pts = best_run
        try:
            return LineString(pts) if len(pts) >= 2 else None
        except Exception:
            return None

    # Heuristic fixes for problematic lanes: smooth gaps and extend/truncate to match side bounds
    def fix_centerline_geometry(center: LineString, left_ls: LineString, right_ls: LineString) -> LineString:
        # If centerline has tiny kinks or duplicates, simplify slightly
        try:
            simplified = center.simplify(0.05, preserve_topology=False)
            if isinstance(simplified, LineString) and len(simplified.coords) >= 2:
                center = simplified
        except Exception:
            pass
        return center

    # For each lane id, compute centerline and boundaries
    for lane_id in lane_ids:
        # Choose boundaries first (required to make centerline from sides)
        left_geom_m, left_tag = choose_side_geom(lane_id, 'left')
        right_geom_m, right_tag = choose_side_geom(lane_id, 'right')
        # Fallback: if one side missing, synthesize from the other by offset
        if left_geom_m is None and right_geom_m is not None:
            try:
                cand = right_geom_m.parallel_offset(args.lane_width, 'left', join_style=2)
                if isinstance(cand, MultiLineString) and cand.geoms:
                    cand = max(cand.geoms, key=lambda ln: ln.length)
                if isinstance(cand, LineString) and not cand.is_empty:
                    left_geom_m = cand
                    left_tag = left_tag or 'solid'
                    if str(lane_id) == '9':
                        print('Info: lane 9 left synthesized from right')
            except Exception:
                pass
        if right_geom_m is None and left_geom_m is not None:
            try:
                cand = left_geom_m.parallel_offset(args.lane_width, 'right', join_style=2)
                if isinstance(cand, MultiLineString) and cand.geoms:
                    cand = max(cand.geoms, key=lambda ln: ln.length)
                if isinstance(cand, LineString) and not cand.is_empty:
                    right_geom_m = cand
                    right_tag = right_tag or 'solid'
                    if str(lane_id) == '9':
                        print('Info: lane 9 right synthesized from left')
            except Exception:
                pass
        if left_geom_m is None or right_geom_m is None:
            continue
        centerline: Optional[LineString] = centerline_from_sides(left_geom_m, right_geom_m, samples=300)
        if centerline is None:
            continue
        # Apply heuristic fix and snap ends inside side envelope
        centerline = fix_centerline_geometry(centerline, left_geom_m, right_geom_m)

        if isinstance(centerline, MultiLineString):
            # Take longest piece
            centerline = max(centerline.geoms, key=lambda ln: ln.length)
        if not isinstance(centerline, LineString) or centerline.is_empty or len(centerline.coords) < 2:
            continue

        # left/right already selected from layers; ensure LineString
        if not isinstance(left_geom_m, LineString) or not isinstance(right_geom_m, LineString):
            continue

        # Convert to lon/lat
        tmp_gdf = gpd.GeoDataFrame(geometry=[centerline, left_geom_m, right_geom_m], crs='EPSG:3857')
        tmp_ll = tmp_gdf.to_crs(epsg=4326)
        center_ll, left_ll, right_ll = [ln for ln in tmp_ll.geometry]

        # Emit OSM ways and relation
        left_way = builder.add_linestring_way(left_ll, {'type': 'line_thin', 'subtype': left_tag or 'solid'})
        right_way = builder.add_linestring_way(right_ll, {'type': 'line_thin', 'subtype': right_tag or 'solid'})
        center_way = builder.add_linestring_way(center_ll, {})
        if left_way and right_way and center_way:
            builder.add_lanelet_relation(left_way, right_way, center_way, lane_id, prefer_relation_id=lane_id)

    # Emit non-path lines for visualization
    def emit_lines(gdf: Optional[gpd.GeoDataFrame], subtype: str):
        if gdf is None or gdf.empty:
            return
        col_is = col(gdf, 'is_path')
        df = gdf
        if col_is and col_is in gdf.columns:
            df = gdf[gdf[col_is].astype(float) != 1.0]
        if df.empty:
            return
        ll_df = df.to_crs(epsg=4326)
        for geom in ll_df.geometry:
            if isinstance(geom, LineString):
                builder.add_linestring_way(geom, {'type': 'line_thin', 'subtype': subtype})
            elif isinstance(geom, MultiLineString):
                for ln in geom.geoms:
                    builder.add_linestring_way(ln, {'type': 'line_thin', 'subtype': subtype})

    emit_lines(dashed_gdf, 'dashed')
    emit_lines(virtual_gdf, 'virtual')
    # Left/right lines that are not path
    if lr_gdf is not None and not lr_gdf.empty:
        col_is_lr = col(lr_gdf, 'is_path')
        df_lr = lr_gdf
        if col_is_lr and col_is_lr in lr_gdf.columns:
            df_lr = lr_gdf[lr_gdf[col_is_lr].astype(float) != 1.0]
        ll_df = df_lr.to_crs(epsg=4326)
        for geom in ll_df.geometry:
            if isinstance(geom, LineString):
                builder.add_linestring_way(geom, {'type': 'line_thin', 'subtype': 'solid'})
            elif isinstance(geom, MultiLineString):
                for ln in geom.geoms:
                    builder.add_linestring_way(ln, {'type': 'line_thin', 'subtype': 'solid'})

    os.makedirs(os.path.dirname(args.output_osm), exist_ok=True)
    builder.save(args.output_osm)


if __name__ == '__main__':
    main()
