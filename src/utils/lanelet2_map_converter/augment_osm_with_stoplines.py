import argparse
import os
from typing import Tuple, List

import geopandas as gpd
from shapely.geometry import LineString, MultiLineString
from lxml import etree


def _read_stopline_coords(stopline_shp_path: str) -> List[List[Tuple[float, float]]]:
    """Read stopline shapefile and return list of line coordinate lists in EPSG:4326 (lon, lat)."""
    if not os.path.exists(stopline_shp_path):
        raise FileNotFoundError(f"Stopline shapefile not found: {stopline_shp_path}")

    gdf = gpd.read_file(stopline_shp_path)
    if gdf.crs is not None and gdf.crs.to_epsg() != 4326:
        gdf = gdf.to_crs("EPSG:4326")

    lines: List[List[Tuple[float, float]]] = []
    for geom in gdf.geometry:
        if geom is None or geom.is_empty:
            continue
        if isinstance(geom, LineString):
            coords = [(float(x), float(y)) for (x, y, *_) in geom.coords]
            if len(coords) >= 2:
                lines.append(coords)
        elif isinstance(geom, MultiLineString):
            for part in geom.geoms:
                if part.is_empty:
                    continue
                coords = [(float(x), float(y)) for (x, y, *_) in part.coords]
                if len(coords) >= 2:
                    lines.append(coords)
        # ignore other geometry types
    return lines


def _find_max_ids(root: etree._Element) -> Tuple[int, int]:
    """Return (max_node_id, max_way_id) parsed as integers; missing treated as 0."""
    max_node = 0
    max_way = 0
    for node in root.findall("node"):
        try:
            max_node = max(max_node, int(node.get("id", "0")))
        except ValueError:
            continue
    for way in root.findall("way"):
        try:
            max_way = max(max_way, int(way.get("id", "0")))
        except ValueError:
            continue
    return max_node, max_way


def _append_stoplines_to_osm(root: etree._Element, stopline_lines: List[List[Tuple[float, float]]]) -> int:
    """Append stopline ways to OSM tree. Return number of ways added."""
    max_node_id, max_way_id = _find_max_ids(root)
    node_id = max_node_id + 1
    way_id = max_way_id + 1

    added = 0
    for coords in stopline_lines:
        # create nodes
        nd_refs: List[str] = []
        for lon, lat in coords:
            nid = str(node_id)
            node_id += 1
            etree.SubElement(root, "node", id=nid, lat=str(lat), lon=str(lon))
            nd_refs.append(nid)
        # create way
        way_elem = etree.SubElement(root, "way", id=str(way_id))
        way_id += 1
        for ref in nd_refs:
            etree.SubElement(way_elem, "nd", ref=ref)
        etree.SubElement(way_elem, "tag", k="type", v="stop_line")
        added += 1
    return added


def augment_osm(input_osm: str, stopline_shp: str, output_osm: str, backup: bool = True) -> int:
    if not os.path.exists(input_osm):
        raise FileNotFoundError(f"Input OSM not found: {input_osm}")
    if not os.path.exists(stopline_shp):
        raise FileNotFoundError(f"Stopline shapefile not found: {stopline_shp}")

    parser = etree.XMLParser(remove_blank_text=True)
    tree = etree.parse(input_osm, parser)
    root = tree.getroot()

    lines = _read_stopline_coords(stopline_shp)
    if not lines:
        return 0

    added = _append_stoplines_to_osm(root, lines)

    if backup and os.path.abspath(input_osm) == os.path.abspath(output_osm):
        bak = input_osm + ".bak"
        if not os.path.exists(bak):
            with open(bak, "wb") as f:
                f.write(etree.tostring(tree, pretty_print=True, xml_declaration=True, encoding="UTF-8"))

    # write output
    tree.write(output_osm, pretty_print=True, xml_declaration=True, encoding="UTF-8")
    return added


def main():
    ap = argparse.ArgumentParser(description="Augment a Lanelet2 OSM by adding stop_line ways from a shapefile")
    ap.add_argument("--input_osm", required=True, help="Path to existing lanelet2_map.osm")
    ap.add_argument("--stopline_shp", required=True, help="Path to stopline shapefile (e.g., stopline.shp)")
    ap.add_argument("--output_osm", required=False, help="Path to write updated OSM (defaults to overwrite input)")
    args = ap.parse_args()

    output_osm = args.output_osm or args.input_osm
    added = augment_osm(args.input_osm, args.stopline_shp, output_osm, backup=True)
    print(f"Added {added} stop_line ways to {output_osm}")


if __name__ == "__main__":
    main()


