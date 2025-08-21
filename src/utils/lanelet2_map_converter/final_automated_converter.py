import argparse
import geopandas as gpd
import numpy as np
from lxml import etree
from shapely.geometry import LineString, Point
from scipy.spatial import KDTree
from collections import defaultdict

# (OsmBuilder class from previous script)
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
        if not points: return None
        node_ids = [self._get_node_id(p) for p in points]
        way_id = self.way_id_counter
        self.way_id_counter -= 1
        self.ways.append({"id": way_id, "nodes": node_ids, "tags": tags or {}})
        return way_id

    def add_relation(self, members, tags=None):
        relation_id = self.relation_id_counter
        self.relation_id_counter -= 1
        self.relations.append({"id": relation_id, "members": members, "tags": tags or {}})
        return relation_id

    def save_osm(self, filename):
        root = etree.Element("osm", version="0.6")
        for coord, node_id in self.nodes.items():
            etree.SubElement(root, "node", id=str(node_id), lat=str(coord[1]), lon=str(coord[0]))
        for way_info in self.ways:
            way_elem = etree.SubElement(root, "way", id=str(way_info["id"]))
            for node_id in way_info["nodes"]: etree.SubElement(way_elem, "nd", ref=str(node_id))
            for k, v in way_info["tags"].items(): etree.SubElement(way_elem, "tag", k=k, v=str(v))
        for rel_info in self.relations:
            rel_elem = etree.SubElement(root, "relation", id=str(rel_info["id"]))
            for member_type, ref, role in rel_info["members"]:
                etree.SubElement(rel_elem, "member", type=member_type, ref=str(ref), role=role)
            for k, v in rel_info["tags"].items(): etree.SubElement(rel_elem, "tag", k=k, v=str(v))
        with open(filename, 'wb') as f:
            f.write(etree.tostring(root, pretty_print=True, xml_declaration=True, encoding='UTF-8'))
        print(f"Saved OSM file to {filename}")


DEFAULT_LANE_WIDTH_M = 3.5

def get_line_direction(line):
    if line.length < 1e-6: return np.array([0.0, 0.0])
    coords = np.array(line.coords)
    vec = coords[-1, :2] - coords[0, :2]
    norm = np.linalg.norm(vec)
    return vec / norm if norm > 1e-6 else np.array([0.0, 0.0])

def main():
    parser = argparse.ArgumentParser(description="Fully automated Lanelet2 converter using a two-pass strategy.")
    parser.add_argument("--boundaries", required=True, help="Path to the boundary network GeoJSON.")
    parser.add_argument("--segments", required=True, help="Path to the exploded Voronoi segments GeoJSON.")
    parser.add_argument("--output_osm", required=True, help="Path for the final output Lanelet2 OSM file.")
    args = parser.parse_args()

    boundary_gdf = gpd.read_file(args.boundaries)
    segments_gdf = gpd.read_file(args.segments)
    osm = OsmBuilder()
    
    # --- PASS 1: Find high-confidence simple lanes ---
    print("--- PASS 1: Finding high-confidence simple lanes... ---")
    
    boundary_points, boundary_line_indices = [], []
    for i, line in enumerate(boundary_gdf.geometry):
        for point in line.coords:
            boundary_points.append(point[:2])
            boundary_line_indices.append(i)
    kdtree = KDTree(boundary_points)

    simple_lanes = []
    processed_segment_indices = set()

    for i, row in segments_gdf.iterrows():
        line = row.geometry
        if line.length < DEFAULT_LANE_WIDTH_M: continue

        sample_points = [line.interpolate(d, normalized=True) for d in np.linspace(0.2, 0.8, 5)]
        consistent_pair_votes = defaultdict(int)
        
        for p in sample_points:
            dists, indices = kdtree.query(p.coords[0][:2], k=10)
            closest_lines = {}
            for dist, idx in zip(dists, indices):
                line_idx = boundary_line_indices[idx]
                if line_idx not in closest_lines: closest_lines[line_idx] = dist
            
            if len(closest_lines) < 2: continue
            sorted_closest = sorted(closest_lines.items(), key=lambda item: item[1])
            idx1, dist1 = sorted_closest[0]; idx2, dist2 = sorted_closest[1]

            if (dist1 + dist2) < DEFAULT_LANE_WIDTH_M * 1.8 and (0.6 < dist1 / (dist2 + 1e-9) < 1.7):
                consistent_pair_votes[tuple(sorted((idx1, idx2)))] += 1
        
        if not consistent_pair_votes: continue
        best_pair = max(consistent_pair_votes, key=consistent_pair_votes.get)
        if consistent_pair_votes[best_pair] >= 4:
            processed_segment_indices.add(i)
            simple_lanes.append({'centerline': line, 'left_id': best_pair[0], 'right_id': best_pair[1]})

    print(f"Found {len(simple_lanes)} high-confidence lanes.")

    # --- PASS 2: Find intersection lanes by connecting simple lanes ---
    print("\n--- PASS 2: Finding intersection lanes... ---")
    
    lane_terminals = {} # Map terminal point WKT to lane index
    for i, lane in enumerate(simple_lanes):
        start_pt, end_pt = Point(lane['centerline'].coords[0]), Point(lane['centerline'].coords[-1])
        lane_terminals[start_pt.wkt] = {'lane_idx': i, 'is_start': True}
        lane_terminals[end_pt.wkt] = {'lane_idx': i, 'is_start': False}
        
    unprocessed_segments = segments_gdf.drop(list(processed_segment_indices))
    intersection_lanes = []

    for _, row in unprocessed_segments.iterrows():
        line = row.geometry
        if line.length < 1.0 or line.length > 50.0: continue
        start_pt, end_pt = Point(line.coords[0]), Point(line.coords[-1])

        start_match, end_match = None, None
        min_dist_start, min_dist_end = float('inf'), float('inf')

        for wkt, terminal_info in lane_terminals.items():
            dist_s = start_pt.distance(Point(eval(wkt.replace('POINT ', ''))))
            if dist_s < min_dist_start:
                min_dist_start = dist_s
                start_match = terminal_info
            
            dist_e = end_pt.distance(Point(eval(wkt.replace('POINT ', ''))))
            if dist_e < min_dist_end:
                min_dist_end = dist_e
                end_match = terminal_info
        
        if min_dist_start < 1.0 and min_dist_end < 1.0 and start_match['lane_idx'] != end_match['lane_idx']:
            in_lane = simple_lanes[start_match['lane_idx']]
            out_lane = simple_lanes[end_match['lane_idx']]
            intersection_lanes.append({'centerline': line, 'in_lane': in_lane, 'out_lane': out_lane})

    print(f"Found {len(intersection_lanes)} potential intersection lanes.")
    
    # --- Final Assembly ---
    print("\n--- Assembling final map... ---")

    for lane in simple_lanes:
        left = boundary_gdf.geometry.iloc[lane['left_id']]
        right = boundary_gdf.geometry.iloc[lane['right_id']]
        
        center_dir = get_line_direction(lane['centerline'])
        p_sample = lane['centerline'].interpolate(0.5, normalized=True)
        if np.cross(center_dir, (np.array(left.interpolate(left.project(p_sample)).coords[0])[:2] - np.array(p_sample.coords[0])[:2])) < 0:
            left = left.reverse()
        if np.cross(center_dir, (np.array(right.interpolate(right.project(p_sample)).coords[0])[:2] - np.array(p_sample.coords[0])[:2])) > 0:
            right = right.reverse()
        
        left_id = osm.add_way(list(left.coords), {"type": "line_thin", "subtype": "solid"})
        right_id = osm.add_way(list(right.coords), {"type": "line_thin", "subtype": "solid"})
        center_id = osm.add_way(list(lane['centerline'].coords), {})
        osm.add_relation([("way", left_id, "left"), ("way", right_id, "right"), ("way", center_id, "centerline")], {"type": "lanelet", "subtype": "road"})
        
    for lane in intersection_lanes:
        in_left = boundary_gdf.geometry.iloc[lane['in_lane']['left_id']]
        in_right = boundary_gdf.geometry.iloc[lane['in_lane']['right_id']]
        out_left = boundary_gdf.geometry.iloc[lane['out_lane']['left_id']]
        out_right = boundary_gdf.geometry.iloc[lane['out_lane']['right_id']]
        
        # Create virtual boundaries
        in_left_end = Point(in_left.coords[-1]) if get_line_direction(in_left) @ get_line_direction(lane['in_lane']['centerline']) > 0 else Point(in_left.coords[0])
        out_left_start = Point(out_left.coords[0]) if get_line_direction(out_left) @ get_line_direction(lane['out_lane']['centerline']) > 0 else Point(out_left.coords[-1])
        virtual_left = LineString([in_left_end, out_left_start])

        in_right_end = Point(in_right.coords[-1]) if get_line_direction(in_right) @ get_line_direction(lane['in_lane']['centerline']) > 0 else Point(in_right.coords[0])
        out_right_start = Point(out_right.coords[0]) if get_line_direction(out_right) @ get_line_direction(lane['out_lane']['centerline']) > 0 else Point(out_right.coords[-1])
        virtual_right = LineString([in_right_end, out_right_start])

        left_id = osm.add_way(list(virtual_left.coords), {"type": "virtual"})
        right_id = osm.add_way(list(virtual_right.coords), {"type": "virtual"})
        center_id = osm.add_way(list(lane['centerline'].coords), {})
        osm.add_relation([("way", left_id, "left"), ("way", right_id, "right"), ("way", center_id, "centerline")], {"type": "lanelet", "subtype": "road"})

    osm.save_osm(args.output_osm)

if __name__ == "__main__":
    main()
