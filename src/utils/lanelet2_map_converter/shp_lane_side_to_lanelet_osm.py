import os
import argparse
from typing import Dict, List, Tuple, Optional

import geopandas as gpd
import numpy as np
from shapely.geometry import LineString, MultiLineString, GeometryCollection, Polygon, MultiPolygon
from shapely.ops import linemerge, unary_union
from lxml import etree


def _to_linestring(geom) -> LineString:
    if geom is None:
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
            parts = list(merged.geoms)
            if not parts:
                return LineString()
            parts.sort(key=lambda g: g.length, reverse=True)
            return parts[0]
    if isinstance(geom, GeometryCollection):
        best = None
        best_len = -1.0
        for g in geom.geoms:
            if isinstance(g, (LineString, MultiLineString)):
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
    dedup: List[Tuple[float, float]] = []
    for c in coords:
        # handle 2D or 3D (x,y[,z])
        x = float(c[0])
        y = float(c[1])
        if not dedup or (x != dedup[-1][0] or y != dedup[-1][1]):
            dedup.append((x, y))
    if len(dedup) < 2:
        return LineString()
    return LineString(dedup)


def _maybe_flip(to_fix: LineString, ref: LineString) -> LineString:
    if to_fix.is_empty or ref.is_empty:
        return to_fix
    ref0 = np.array(ref.coords[0])
    to0 = np.array(to_fix.coords[0])
    toE = np.array(to_fix.coords[-1])
    d_same_len = np.linalg.norm(ref0 - to0)
    d_flip_len = np.linalg.norm(ref0 - toE)
    return to_fix if d_same_len <= d_flip_len else LineString(list(to_fix.coords)[::-1])


class OsmIds:
    def __init__(self) -> None:
        self.next_node = 1
        self.next_way = 1
        self.next_rel = 1

    def n(self) -> int:
        i = self.next_node
        self.next_node += 1
        return i

    def w(self) -> int:
        i = self.next_way
        self.next_way += 1
        return i

    def r(self) -> int:
        i = self.next_rel
        self.next_rel += 1
        return i


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
        self.rels[rid] = {'members': list(members), 'tags': dict(tags)}
        return rid

    def to_etree(self) -> etree._Element:
        root = etree.Element('osm', version='0.6', generator='shp_lane_side_to_lanelet_osm')
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


def _merge_side(lines: List[LineString]) -> LineString:
    lines = [l for l in lines if isinstance(l, LineString) and not l.is_empty]
    if not lines:
        return LineString()
    try:
        merged = linemerge(lines)
    except Exception:
        merged = lines[0] if len(lines) == 1 else LineString()
    if isinstance(merged, LineString):
        return _clean_linestring(merged)
    if isinstance(merged, MultiLineString):
        parts = list(merged.geoms)
        parts.sort(key=lambda g: g.length, reverse=True)
        return _clean_linestring(parts[0]) if parts else LineString()
    return _clean_linestring(merged)


def _centerline_from_lr(left: LineString, right: LineString, samples: int = 100) -> LineString:
    if left.is_empty or right.is_empty:
        return LineString()
    # 간단 등분 샘플 기반 중선(좌우 길이 차이를 고려해 비례 샘플)
    n = max(2, samples)
    pts: List[Tuple[float, float]] = []
    for i in range(n):
        t = i / (n - 1)
        dl = t * left.length
        dr = t * right.length
        pl = left.interpolate(dl)
        pr = right.interpolate(dr)
        pts.append(((pl.x + pr.x) / 2.0, (pl.y + pr.y) / 2.0))
    return _clean_linestring(LineString(pts))


def _read_shp(path: str, crs_out: Optional[str]) -> Optional[gpd.GeoDataFrame]:
    if not path or not os.path.exists(path):
        return None
    try:
        gdf = gpd.read_file(path)
        if crs_out:
            gdf = gdf.to_crs(crs_out)
        else:
            if gdf.crs is None or (getattr(gdf.crs, 'to_epsg', lambda: None)() != 4326):
                gdf = gdf.to_crs('EPSG:4326')
        return gdf
    except Exception:
        return None


def _add_lines_osm(osm: Osm, gdf: gpd.GeoDataFrame, type_tag: str, subtype: Optional[str] = None) -> int:
    if gdf is None or gdf.empty:
        return 0
    count = 0
    for _, row in gdf.iterrows():
        geom = row.geometry
        ls = _clean_linestring(geom)
        if ls.is_empty:
            # try to salvage from multilines or geometry collections
            if isinstance(geom, (MultiLineString, GeometryCollection)):
                ls = _to_linestring(geom)
        if ls.is_empty:
            continue
        tags = {'type': type_tag}
        if subtype:
            tags['subtype'] = subtype
        wid = osm.way(list(ls.coords), tags)
        if wid > 0:
            count += 1
    return count


def _add_polygons_osm(osm: Osm, gdf: gpd.GeoDataFrame, type_tag: str, subtype: Optional[str] = None) -> int:
    if gdf is None or gdf.empty:
        return 0
    count = 0
    for _, row in gdf.iterrows():
        geom = row.geometry
        poly: Optional[Polygon] = None
        if isinstance(geom, Polygon):
            poly = geom
        elif isinstance(geom, MultiPolygon):
            # choose largest polygon by area
            polys = list(geom.geoms)
            if polys:
                poly = max(polys, key=lambda p: p.area)
        if poly is None or poly.is_empty:
            continue
        coords = list(poly.exterior.coords)
        tags = {'type': type_tag}
        if subtype:
            tags['subtype'] = subtype
        wid = osm.way(coords, tags)
        if wid > 0:
            count += 1
    return count


def _extract_lines(geom) -> List[LineString]:
    lines: List[LineString] = []
    if geom is None:
        return lines
    if isinstance(geom, LineString):
        ls = _clean_linestring(geom)
        if not ls.is_empty:
            lines.append(ls)
    elif isinstance(geom, MultiLineString):
        for g in geom.geoms:
            ls = _clean_linestring(g)
            if not ls.is_empty:
                lines.append(ls)
    elif isinstance(geom, GeometryCollection):
        for g in geom.geoms:
            lines.extend(_extract_lines(g))
    return lines


def build_from_lane_side_shp(input_shp: str, output_osm: str, id_field: str = 'lanelet_id', side_field: str = 'lane_side', crs_out: Optional[str] = None) -> None:
    gdf = gpd.read_file(input_shp)
    # 좌표계를 OSM 표준(WGS84 경위도)로 변환. 사용자가 지정하면 해당 CRS로, 아니면 EPSG:4326 강제.
    try:
        if crs_out:
            gdf = gdf.to_crs(crs_out)
        else:
            if gdf.crs is None or (getattr(gdf.crs, 'to_epsg', lambda: None)() != 4326):
                gdf = gdf.to_crs('EPSG:4326')
    except Exception:
        # 재투영 실패 시에도 진행은 하되, 좌표가 미터계라면 lanelet2 파서가 실패할 수 있음
        pass

    # 열 이름 정규화(대소문자 혼합 대응)
    cols = {c.lower(): c for c in gdf.columns}
    id_col = cols.get(id_field.lower())
    side_col = cols.get(side_field.lower())
    geom_col = gdf.geometry.name

    if id_col is None or side_col is None:
        raise RuntimeError(f"Required columns not found: id='{id_field}', side='{side_field}'")

    groups: Dict[str, Dict[str, List[LineString]]] = {}
    for _, row in gdf.iterrows():
        lid_val = row[id_col]
        side_val = str(row[side_col]).strip().lower()
        geom = _clean_linestring(row[geom_col])
        if geom.is_empty:
            continue
        if side_val not in ('left', 'right', 'center'):
            continue
        key = str(int(lid_val)) if isinstance(lid_val, (int, np.integer)) else str(lid_val)
        groups.setdefault(key, {'left': [], 'right': [], 'center': []})
        groups[key][side_val].append(geom)

    osm = Osm()
    for lanelet_id, sides in groups.items():
        left_m = _merge_side(sides['left'])
        right_m = _merge_side(sides['right'])
        center_m = _merge_side(sides['center'])

        # 좌/우 둘 다 있으면 방향 정규화 및 센터 유도
        if not left_m.is_empty and not right_m.is_empty:
            right_m = _maybe_flip(right_m, left_m)
            if center_m.is_empty:
                center_m = _centerline_from_lr(left_m, right_m, samples=120)

        # 센터만 있는 경우도 허용(좌/우는 비어있을 수 있음)
        # 단, lanelet relation 생성을 위해서는 좌/우 최소 하나 이상이 있는 편이 시각화에 유리
        # 여기서는 가능한 태그를 최대한 넣되, 없는 멤버는 생략

        way_members: List[Tuple[str, int, str]] = []
        left_w = right_w = center_w = -1
        if not left_m.is_empty:
            left_w = osm.way(list(left_m.coords), {'type': 'line_thin', 'subtype': 'solid'})
            if left_w > 0:
                way_members.append(('way', left_w, 'left'))
        if not right_m.is_empty:
            right_w = osm.way(list(right_m.coords), {'type': 'line_thin', 'subtype': 'solid'})
            if right_w > 0:
                way_members.append(('way', right_w, 'right'))
        if not center_m.is_empty:
            center_w = osm.way(list(center_m.coords), {})
            if center_w > 0:
                way_members.append(('way', center_w, 'centerline'))

        if not way_members:
            continue

        rel_tags = {'type': 'lanelet', 'subtype': 'road', 'id': str(lanelet_id)}
        osm.rel_fixed_id(int(lanelet_id) if lanelet_id.isdigit() else osm.ids.r(), way_members, rel_tags)

    osm.save(output_osm)


def build_kcity_extras(osm_path: str, extras_dir: str, crs_out: Optional[str]) -> None:
    """
    Append additional layers (dashed_line, left_right_line, crosswalk, roadborder, stopline, Markings, dashed_markings)
    to an existing OSM file by loading it, adding ways, and saving back.
    """
    # Load existing OSM
    # Minimal XML manipulation: parse then append new ways and tags via Osm helper by re-parsing is complex,
    # so instead regenerate an Osm structure is heavy. Simpler: read OSM as text is not robust.
    # Here, we will re-open the OSM into an Osm instance by parsing nodes/ways is non-trivial; therefore,
    # we will extend by creating a new Osm and merging is out of scope.
    # Instead, we will create a sibling file with suffix and then replace original.
    base_xml = etree.parse(osm_path).getroot()
    next_way_id = 1
    for w in base_xml.findall('way'):
        wid = int(w.get('id', '0'))
        if wid >= next_way_id:
            next_way_id = wid + 1
    # nodes map for de-duplication
    nodes_el = base_xml.findall('node')
    node_map = {(float(n.get('lon')), float(n.get('lat'))): int(n.get('id')) for n in nodes_el}

    def ensure_node(lon: float, lat: float) -> int:
        key = (round(lon, 7), round(lat, 7))
        for (k_lon, k_lat), nid in list(node_map.items()):
            if round(k_lon, 7) == key[0] and round(k_lat, 7) == key[1]:
                return nid
        nid = max(node_map.values()) + 1 if node_map else 1
        node_map[(key[0], key[1])] = nid
        el = etree.SubElement(base_xml, 'node', id=str(nid), lon=str(key[0]), lat=str(key[1]))
        return nid

    def append_way(coords: List[Tuple[float, float]], tags: Dict[str, str]) -> None:
        nonlocal next_way_id
        if not coords or len(coords) < 2:
            return
        w_el = etree.SubElement(base_xml, 'way', id=str(next_way_id))
        for x, y in coords:
            nid = ensure_node(x, y)
            etree.SubElement(w_el, 'nd', ref=str(nid))
        for k, v in tags.items():
            etree.SubElement(w_el, 'tag', k=str(k), v=str(v))
        next_way_id += 1

    def add_lines_from_file(shp_name: str, type_tag: str, subtype: Optional[str] = None, viz_line: bool = False) -> int:
        shp_path = os.path.join(extras_dir, shp_name)
        gdf = _read_shp(shp_path, crs_out)
        if gdf is None or gdf.empty:
            return 0
        cnt = 0
        for _, row in gdf.iterrows():
            geom = row.geometry
            line_list = _extract_lines(geom)
            for ls in line_list:
                if ls.is_empty:
                    continue
                if viz_line:
                    tags = {'type': 'line_thin'}
                    tags['subtype'] = subtype if subtype else type_tag
                else:
                    tags = {'type': type_tag}
                    if subtype:
                        tags['subtype'] = subtype
                append_way(list(ls.coords), tags)
                cnt += 1
        return cnt

    def add_polys_from_file(shp_name: str, type_tag: str, subtype: Optional[str] = None, viz_line: bool = False) -> int:
        shp_path = os.path.join(extras_dir, shp_name)
        gdf = _read_shp(shp_path, crs_out)
        if gdf is None or gdf.empty:
            return 0
        cnt = 0
        for _, row in gdf.iterrows():
            geom = row.geometry
            poly = None
            if isinstance(geom, Polygon):
                poly = geom
            elif isinstance(geom, MultiPolygon):
                polys = list(geom.geoms)
                if polys:
                    poly = max(polys, key=lambda p: p.area)
            if poly is None or poly.is_empty:
                continue
            if viz_line:
                tags = {'type': 'line_thin'}
                tags['subtype'] = subtype if subtype else type_tag
            else:
                tags = {'type': type_tag}
                if subtype:
                    tags['subtype'] = subtype
            append_way(list(poly.exterior.coords), tags)
            cnt += 1
        return cnt

    def add_stoplines_from_file(shp_name: str) -> int:
        """Add stop lines with ref mapping and type tags.
        Expected columns:
          - geometry: LineString/MultiLineString
          - ref lanelet id in one of: ref_lane, ref_lanelet_id, lanelet_id
          - stopline type in one of: stopline_type, sl_type, type, subtype (preferred order)
        Writes tags: type=line_thin, subtype=stop_line, ref_lanelet_id=<id>, stopline_type=<value>
        """
        # Try common filename variants to be robust to naming differences
        candidates = [
            shp_name,
            'stopline.shp', 'Stopline.shp', 'STOPLINE.SHP',
            'stop_line.shp', 'Stop_Line.shp', 'Stop_Line.SHP'
        ]
        gdf = None
        shp_path = None
        for nm in candidates:
            p = os.path.join(extras_dir, nm)
            gdf = _read_shp(p, crs_out)
            if gdf is not None and not gdf.empty:
                shp_path = p
                break
        if gdf is None or gdf.empty:
            return 0
        # normalize column names for lookup
        cols = {c.lower(): c for c in gdf.columns}
        ref_cols = [c for c in ['ref_lane', 'ref_lanelet_id', 'lanelet_id'] if c in cols]
        type_cols = [c for c in ['stopline_type', 'sl_type', 'type', 'subtype'] if c in cols]
        cnt = 0
        for _, row in gdf.iterrows():
            geom = row.geometry
            line_list = _extract_lines(geom)
            # read ref id
            ref_id = None
            for rc in ref_cols:
                try:
                    val = row[cols[rc]]
                    if val is not None and str(val) != '':
                        ref_id = int(val)
                        break
                except Exception:
                    continue
            # read stopline type string
            stype = None
            for tc in type_cols:
                try:
                    v = row[cols[tc]]
                    if v is not None and str(v) != '':
                        stype = str(v)
                        break
                except Exception:
                    continue
            for ls in line_list:
                if ls.is_empty:
                    continue
                w_el_tags = {'type': 'line_thin', 'subtype': 'stop_line'}
                if ref_id is not None:
                    w_el_tags['ref_lanelet_id'] = str(ref_id)
                if stype is not None:
                    w_el_tags['stopline_type'] = stype
                append_way(list(ls.coords), w_el_tags)
                cnt += 1
        return cnt

    def add_parking_paths(shp_name: str) -> int:
        """Add parking paths grouped by parking_id so each path_id appears ONCE.
        For each parking_id group, merge segments to a single LineString (best-effort):
          1) Visualization way: type=line_thin, subtype=parking_path
          2) Planner way: type=parking_path, path_id=<parking_id>
        If no explicit id column, uses sequential ids starting from 1.
        """
        shp_path = os.path.join(extras_dir, shp_name)
        gdf = _read_shp(shp_path, crs_out)
        if gdf is None or gdf.empty:
            return 0

        # normalize columns (prefer parking_id explicitly)
        cols = {c.lower(): c for c in gdf.columns}
        path_id_col = None
        for c in ('parking_id', 'path_id', 'id', 'name'):
            if c in cols:
                path_id_col = cols[c]
                break

        # group by path id value
        groups: Dict[str, List[LineString]] = {}
        seq = 1
        for _, row in gdf.iterrows():
            # determine id
            pid_val: Optional[str] = None
            if path_id_col is not None:
                try:
                    v = row[path_id_col]
                    if v is not None and str(v) != '':
                        # normalize e.g., 1.0 -> 1
                        if isinstance(v, (int, np.integer)):
                            pid_val = str(int(v))
                        else:
                            try:
                                fv = float(v)
                                if abs(fv - round(fv)) < 1e-6:
                                    pid_val = str(int(round(fv)))
                                else:
                                    pid_val = str(v)
                            except Exception:
                                pid_val = str(v)
                except Exception:
                    pid_val = None
            if pid_val is None:
                pid_val = str(seq)
                seq += 1
            # collect lines
            line_list = _extract_lines(row.geometry)
            if not line_list:
                continue
            groups.setdefault(pid_val, [])
            groups[pid_val].extend(line_list)

        # merge and write once per id
        cnt = 0
        for pid_val, lines in groups.items():
            # try merging segments into a single LineString, robustly handle geometry types
            final_ls: LineString = LineString()
            try:
                u = unary_union(lines)
                if isinstance(u, LineString):
                    final_ls = _clean_linestring(u)
                elif isinstance(u, MultiLineString):
                    lm = linemerge(u)
                    if isinstance(lm, LineString):
                        final_ls = _clean_linestring(lm)
                    elif isinstance(lm, MultiLineString):
                        parts = list(lm.geoms)
                        parts.sort(key=lambda g: g.length, reverse=True)
                        final_ls = _clean_linestring(parts[0]) if parts else LineString()
                else:
                    # fallback to longest of input lines
                    parts = list(lines)
                    parts.sort(key=lambda g: g.length, reverse=True)
                    if parts:
                        final_ls = _clean_linestring(parts[0])
            except Exception:
                # fallback: if single line, use it; else try linemerge on MultiLineString
                if len(lines) == 1:
                    final_ls = _clean_linestring(lines[0])
                else:
                    try:
                        lm = linemerge(MultiLineString(lines))
                        if isinstance(lm, LineString):
                            final_ls = _clean_linestring(lm)
                        elif isinstance(lm, MultiLineString):
                            parts = list(lm.geoms)
                            parts.sort(key=lambda g: g.length, reverse=True)
                            final_ls = _clean_linestring(parts[0]) if parts else LineString()
                    except Exception:
                        parts = list(lines)
                        parts.sort(key=lambda g: g.length, reverse=True)
                        if parts:
                            final_ls = _clean_linestring(parts[0])
            if final_ls.is_empty:
                continue
            coords = list(final_ls.coords)
            # 1) Visualization
            viz_tags = {'type': 'line_thin', 'subtype': 'parking_path'}
            append_way(coords, viz_tags)
            # 2) Planner
            plan_tags = {'type': 'parking_path', 'path_id': str(pid_val)}
            append_way(coords, plan_tags)
            cnt += 1
        return cnt

    # Map known layers → lanelet2 types used by yabloc ll2_decomposer
    added = 0
    # 시각화 호환(viz_line=True)로 넣으면 visualizer에서도 보이고, ll2_decomposer는 line_thin도 수집
    added += add_lines_from_file('dashed_line.shp', 'dashed_line', viz_line=True)
    added += add_lines_from_file('left_right_line.shp', 'left_right_line', viz_line=True)
    added += add_lines_from_file('roadborder.shp', 'road_border', viz_line=True)
    # 주차 사전 경로 시각화를 위한 parking_path 레이어 추가
    # 주차 경로: 시각화(line_thin/subtype)와 플래너용(type=parking_path,path_id) 모두 추가
    added += add_parking_paths('parking_path.shp')
    added += add_stoplines_from_file('stopline.shp')
    added += add_lines_from_file('Markings.shp', 'Markings', viz_line=True)
    # dashed_markings: 선 또는 폴리곤 모두 지원
    cnt_dm = add_lines_from_file('dashed_markings.shp', 'dashed', viz_line=True)
    if cnt_dm == 0:
        added += add_polys_from_file('dashed_markings.shp', 'dashed', viz_line=True)
    else:
        added += cnt_dm
    # crosswalk: 선/폴리곤 모두 지원(표준 표시는 폴리곤 외곽선)
    cnt_cw = add_lines_from_file('crosswalk.shp', 'crosswalk', viz_line=True)
    if cnt_cw == 0:
        added += add_polys_from_file('crosswalk.shp', 'crosswalk', viz_line=True)
    else:
        added += cnt_cw

    # Save back
    xml_bytes = etree.tostring(base_xml, pretty_print=True, xml_declaration=True, encoding='UTF-8')
    with open(osm_path, 'wb') as f:
        f.write(xml_bytes)


def main():
    p = argparse.ArgumentParser(description='Convert SHP with lanelet_id/lane_side(left,right,center) to Lanelet2 OSM')
    p.add_argument('--input', required=True, help='Input shapefile path')
    p.add_argument('--output', required=True, help='Output OSM path')
    p.add_argument('--crs-out', default=None, help='Optional: reproject to CRS before writing (e.g., epsg:4326)')
    p.add_argument('--extras-dir', default=None, help='Optional: directory containing extra shapefiles (kcity_v7/shp)')
    p.add_argument('--id-field', default='lanelet_id')
    p.add_argument('--side-field', default='lane_side')
    args = p.parse_args()

    build_from_lane_side_shp(args.input, args.output, id_field=args.id_field, side_field=args.side_field, crs_out=args.crs_out)
    if args.extras_dir:
        build_kcity_extras(args.output, args.extras_dir, args.crs_out)


if __name__ == '__main__':
    main()


