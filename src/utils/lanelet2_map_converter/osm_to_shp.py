import os
import argparse
from typing import Dict, List, Tuple
from lxml import etree
import geopandas as gpd
from shapely.geometry import LineString, Point


def parse_osm(osm_path: str):
    tree = etree.parse(osm_path)
    root = tree.getroot()

    nodes: Dict[str, Tuple[float, float]] = {}
    ways: List[Dict] = []

    for n in root.findall('node'):
        nid = n.get('id')
        lat = float(n.get('lat'))
        lon = float(n.get('lon'))
        nodes[nid] = (lon, lat)

    for w in root.findall('way'):
        nds = [nd.get('ref') for nd in w.findall('nd')]
        tags = {t.get('k'): t.get('v') for t in w.findall('tag')}
        ways.append({'nodes': nds, 'tags': tags})

    rels: List[Dict] = []
    for r in root.findall('relation'):
        members = [(m.get('type'), m.get('ref'), m.get('role')) for m in r.findall('member')]
        tags = {t.get('k'): t.get('v') for t in r.findall('tag')}
        rels.append({'members': members, 'tags': tags})

    return nodes, ways, rels


def build_linestring(nodes: Dict[str, Tuple[float, float]], node_ids: List[str]) -> LineString:
    coords: List[Tuple[float, float]] = []
    for nid in node_ids:
        if nid not in nodes:
            continue
        coords.append(nodes[nid])
    # drop duplicates in sequence
    dedup: List[Tuple[float, float]] = []
    for c in coords:
        if not dedup or c != dedup[-1]:
            dedup.append(c)
    return LineString(dedup) if len(dedup) >= 2 else LineString()


def export_shapefiles(osm_path: str, out_dir: str) -> None:
    os.makedirs(out_dir, exist_ok=True)
    nodes, ways, rels = parse_osm(osm_path)

    centerline_feats = []
    boundary_feats = []
    stopline_feats = []
    marking_feats = []
    other_feats = []

    for w in ways:
        geom = build_linestring(nodes, w['nodes'])
        tags = w['tags']
        t = tags.get('type', '')
        subtype = tags.get('subtype', '')

        row = {'type': t, 'subtype': subtype, 'geometry': geom}

        if t == 'lanelet' and 'centerline' in subtype:
            centerline_feats.append(row)
        elif t in ('line_thin', 'line_thick'):
            boundary_feats.append(row)
        elif t == 'stop_line':
            stopline_feats.append(row)
        elif t in ('road_marking', 'surface_mark', 'crosswalk_line'):
            marking_feats.append(row)
        else:
            other_feats.append(row)

    def to_gdf(rows):
        if not rows:
            return gpd.GeoDataFrame(columns=['type', 'subtype', 'geometry'], geometry='geometry', crs='EPSG:4326')
        return gpd.GeoDataFrame(rows, geometry='geometry', crs='EPSG:4326')

    gdf_center = to_gdf(centerline_feats)
    gdf_bound = to_gdf(boundary_feats)
    gdf_stop = to_gdf(stopline_feats)
    gdf_mark = to_gdf(marking_feats)
    gdf_other = to_gdf(other_feats)

    # Save individual layers; QGIS can open the folder as a set of layers
    if len(gdf_center) > 0:
        gdf_center.to_file(os.path.join(out_dir, 'centerline.shp'))
    if len(gdf_bound) > 0:
        gdf_bound.to_file(os.path.join(out_dir, 'boundary.shp'))
    if len(gdf_stop) > 0:
        gdf_stop.to_file(os.path.join(out_dir, 'stopline.shp'))
    if len(gdf_mark) > 0:
        gdf_mark.to_file(os.path.join(out_dir, 'markings.shp'))
    if len(gdf_other) > 0:
        gdf_other.to_file(os.path.join(out_dir, 'others.shp'))


def main():
    p = argparse.ArgumentParser(description='Convert Lanelet2 OSM to Shapefiles for QGIS')
    p.add_argument('--input-osm', required=True)
    p.add_argument('--output-dir', required=True)
    args = p.parse_args()

    export_shapefiles(args.input_osm, args.output_dir)


if __name__ == '__main__':
    main()


