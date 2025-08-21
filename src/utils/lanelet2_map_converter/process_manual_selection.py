import argparse
import geopandas as gpd
import numpy as np
from lxml import etree
from shapely.geometry import LineString, Point

class OsmBuilder:
    def __init__(self):
        self.node_id_counter = -1
        self.way_id_counter = -1
        self.relation_id_counter = -1
        self.nodes = {}
        self.ways = []
        self.relations = []

    def _get_node_id(self, point):
        coord = (point[0], point[1])
        if coord not in self.nodes:
            self.nodes[coord] = self.node_id_counter
            self.node_id_counter -= 1
        return self.nodes[coord]

    def add_way(self, points, tags=None):
        if not points:
            return None
        
        node_ids = [self._get_node_id(p) for p in points]
        way_id = self.way_id_counter
        self.way_id_counter -= 1
        
        way_info = {"id": way_id, "nodes": node_ids, "tags": tags or {}}
        self.ways.append(way_info)
        return way_id

    def add_relation(self, members, tags=None):
        relation_id = self.relation_id_counter
        self.relation_id_counter -= 1
        
        relation_info = {
            "id": relation_id,
            "members": members, # e.g., [("way", way_id, "role")]
            "tags": tags or {}
        }
        self.relations.append(relation_info)
        return relation_id

    def save_osm(self, filename):
        root = etree.Element("osm", version="0.6")

        for coord, node_id in self.nodes.items():
            etree.SubElement(root, "node", id=str(node_id), lat=str(coord[1]), lon=str(coord[0]))

        for way_info in self.ways:
            way_elem = etree.SubElement(root, "way", id=str(way_info["id"]))
            for node_id in way_info["nodes"]:
                etree.SubElement(way_elem, "nd", ref=str(node_id))
            for k, v in way_info["tags"].items():
                etree.SubElement(way_elem, "tag", k=k, v=str(v))
        
        for rel_info in self.relations:
            rel_elem = etree.SubElement(root, "relation", id=str(rel_info["id"]))
            for member_type, ref, role in rel_info["members"]:
                etree.SubElement(rel_elem, "member", type=member_type, ref=str(ref), role=role)
            for k, v in rel_info["tags"].items():
                etree.SubElement(rel_elem, "tag", k=k, v=str(v))
        
        with open(filename, 'wb') as f:
            f.write(etree.tostring(root, pretty_print=True, xml_declaration=True, encoding='UTF-8'))
        print(f"Saved OSM file to {filename}")


def get_line_direction(line):
    if line.length < 1e-6: return np.array([0, 0])
    coords = np.array(line.coords)
    return (coords[-1, :2] - coords[0, :2]) / np.linalg.norm(coords[-1, :2] - coords[0, :2])

def create_virtual_boundary(centerline, in_line, out_line):
    """Creates a virtual boundary by connecting the correct endpoints of in and out lines."""
    center_start = Point(centerline.coords[0])
    center_end = Point(centerline.coords[-1])

    in_p1, in_p2 = Point(in_line.coords[0]), Point(in_line.coords[-1])
    out_p1, out_p2 = Point(out_line.coords[0]), Point(out_line.coords[-1])

    start_node = in_p1 if center_start.distance(in_p1) < center_start.distance(in_p2) else in_p2
    end_node = out_p1 if center_end.distance(out_p1) < center_end.distance(out_p2) else out_p2
    
    return LineString([start_node, end_node])

def main():
    parser = argparse.ArgumentParser(description="Generate Lanelet2 map from manually selected data (handles simple roads and intersections).")
    parser.add_argument("--manual_selection", required=True, help="Path to the GeoJSON file with selected centerlines and boundary IDs.")
    parser.add_argument("--boundary_network", required=True, help="Path to the debug_boundary_network.geojson file.")
    parser.add_argument("--output_osm", required=True, help="Path for the final output Lanelet2 OSM file.")
    args = parser.parse_args()

    manual_gdf = gpd.read_file(args.manual_selection)
    boundary_gdf = gpd.read_file(args.boundary_network)
    osm = OsmBuilder()

    for index, row in manual_gdf.iterrows():
        centerline = row.geometry
        left_boundary, right_boundary = None, None

        # Case 1: Simple Road (left_id and right_id are present)
        if 'left_id' in row and row['left_id'] is not None and str(row['left_id']).isdigit():
            left_id = int(row['left_id'])
            right_id = int(row['right_id'])
            left_boundary = boundary_gdf.geometry.iloc[left_id]
            right_boundary = boundary_gdf.geometry.iloc[right_id]
            
            # Orient boundaries correctly
            center_dir = get_line_direction(centerline)
            p_sample = centerline.interpolate(0.5, normalized=True)
            if np.cross(center_dir, (np.array(left_boundary.interpolate(left_boundary.project(p_sample)).coords[0])[:2] - np.array(p_sample.coords[0])[:2])) < 0:
                left_boundary = left_boundary.reverse()
            if np.cross(center_dir, (np.array(right_boundary.interpolate(right_boundary.project(p_sample)).coords[0])[:2] - np.array(p_sample.coords[0])[:2])) > 0:
                right_boundary = right_boundary.reverse()

        # Case 2: Intersection (in/out IDs are present)
        elif 'in_left_id' in row and row['in_left_id'] is not None and str(row['in_left_id']).isdigit():
            in_left = boundary_gdf.geometry.iloc[int(row['in_left_id'])]
            out_left = boundary_gdf.geometry.iloc[int(row['out_left_id'])]
            in_right = boundary_gdf.geometry.iloc[int(row['in_right_id'])]
            out_right = boundary_gdf.geometry.iloc[int(row['out_right_id'])]

            # Create virtual boundaries
            left_boundary = create_virtual_boundary(centerline, in_left, out_left)
            right_boundary = create_virtual_boundary(centerline, in_right, out_right)
        
        else:
            print(f"Skipping row {index}: No valid boundary IDs found.")
            continue

        # Add to OSM
        left_way_id = osm.add_way(list(left_boundary.coords), {"type": "line_thin", "subtype": "solid", "color": "white"})
        right_way_id = osm.add_way(list(right_boundary.coords), {"type": "line_thin", "subtype": "solid", "color": "white"})
        if 'in_left_id' in row and row['in_left_id'] is not None and str(row['in_left_id']).isdigit():
             left_way_id = osm.add_way(list(left_boundary.coords), {"type": "virtual", "color": "blue"})
             right_way_id = osm.add_way(list(right_boundary.coords), {"type": "virtual", "color": "blue"})


        centerline_way_id = osm.add_way(list(centerline.coords), {})

        if left_way_id and right_way_id and centerline_way_id:
            osm.add_relation(
                members=[("way", left_way_id, "left"), ("way", right_way_id, "right"), ("way", centerline_way_id, "centerline")],
                tags={"type": "lanelet", "subtype": "road"}
            )

    osm.save_osm(args.output_osm)
    print("Lanelet2 map generation complete.")

if __name__ == "__main__":
    main()
