import argparse
import geopandas as gpd
import numpy as np
from lxml import etree
from shapely.geometry import LineString, Point, MultiLineString
from shapely.ops import unary_union
from scipy.spatial import KDTree
from collections import defaultdict
import networkx as nx

# (OsmBuilder class from previous script)
class OsmBuilder:
    # ... [The full OsmBuilder class code remains unchanged. It is omitted here for brevity.] ...
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
        way_id = self.way_id_counter; self.way_id_counter -= 1
        self.ways.append({"id": way_id, "nodes": node_ids, "tags": tags or {}})
        return way_id
    def add_relation(self, members, tags=None):
        relation_id = self.relation_id_counter; self.relation_id_counter -= 1
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
SNAP_THRESHOLD = 0.1

def score_segment(segment, boundary_kdtree, boundary_line_indices):
    """Calculates a quality score for a segment. Lower is better."""
    if segment.length < 1.0: return float('inf'), None
    
    sample_points = [segment.interpolate(d, normalized=True) for d in np.linspace(0.2, 0.8, 5)]
    variances = []
    widths = []
    boundary_pairs = []

    for p in sample_points:
        dists, indices = boundary_kdtree.query(p.coords[0][:2], k=10)
        closest_lines = {}
        for dist, idx in zip(dists, indices):
            line_idx = boundary_line_indices[idx]
            if line_idx not in closest_lines: closest_lines[line_idx] = dist
        
        if len(closest_lines) < 2: continue
        sorted_closest = sorted(closest_lines.items(), key=lambda item: item[1])
        idx1, dist1 = sorted_closest[0]; idx2, dist2 = sorted_closest[1]

        width = dist1 + dist2
        if not (DEFAULT_LANE_WIDTH_M * 0.5 < width < DEFAULT_LANE_WIDTH_M * 2.0): continue

        variances.append(abs(dist1 / (dist2 + 1e-9) - 1))
        widths.append(width)
        boundary_pairs.append(tuple(sorted((idx1, idx2))))

    if not variances: return float('inf'), None

    # Find the most consistent boundary pair
    if not boundary_pairs: return float('inf'), None
    most_common_pair = max(set(boundary_pairs), key=boundary_pairs.count)

    # High penalty if the boundary pair is not consistent
    consistency_penalty = 1 / (boundary_pairs.count(most_common_pair) / len(boundary_pairs))
    
    score = (np.mean(variances) + np.std(widths)) * consistency_penalty
    return score, most_common_pair

def main():
    parser = argparse.ArgumentParser(description="Fully automated Lanelet2 converter using Graph-Based Pathfinding.")
    parser.add_argument("--boundaries", required=True)
    parser.add_argument("--segments", required=True)
    parser.add_argument("--output_osm", required=True)
    args = parser.parse_args()

    print("Loading data...")
    boundary_gdf = gpd.read_file(args.boundaries)
    segments_gdf = gpd.read_file(args.segments)
    osm = OsmBuilder()

    # --- 1. Build Scored Graph ---
    print("--- Building and scoring road network graph... ---")
    G = nx.Graph()
    boundary_pts, boundary_indices = [], []
    for i, line in enumerate(boundary_gdf.geometry):
        for point in line.coords:
            boundary_pts.append(point[:2])
            boundary_indices.append(i)
    boundary_kdtree = KDTree(boundary_pts)
    
    # Snap endpoints to create nodes
    endpoint_kdtree = KDTree([p for seg in segments_gdf.geometry for p in (seg.coords[0], seg.coords[-1])])
    all_endpoints = list(endpoint_kdtree.data)
    
    for i, seg in enumerate(segments_gdf.geometry):
        start_idx = endpoint_kdtree.query(seg.coords[0])[1]
        end_idx = endpoint_kdtree.query(seg.coords[-1])[1]
        u, v = tuple(all_endpoints[start_idx]), tuple(all_endpoints[end_idx])

        if u == v: continue

        score, pair = score_segment(seg, boundary_kdtree, boundary_indices)
        if score < 2.0: # Quality threshold
            G.add_edge(u, v, weight=score, segment=seg, pair=pair, id=i)

    print(f"Graph built with {G.number_of_nodes()} nodes and {G.number_of_edges()} high-quality edges.")

    # --- 2. Find Best Paths ---
    print("--- Tracing best paths through graph... ---")
    paths = []
    visited_edges = set()

    # Find nodes that are endpoints of the road network
    end_nodes = [n for n, deg in G.degree() if deg == 1]
    
    for start_node in end_nodes:
        if any(start_node in path for path in paths): continue

        current_path = [start_node]
        current_node = start_node
        
        while G.degree(current_node) != 0:
            best_neighbor = None
            min_weight = float('inf')
            
            is_dead_end = True
            for neighbor in G.neighbors(current_node):
                edge_id = G.get_edge_data(current_node, neighbor)['id']
                if edge_id in visited_edges: continue
                is_dead_end = False
                
                weight = G.get_edge_data(current_node, neighbor)['weight']
                if weight < min_weight:
                    min_weight = weight
                    best_neighbor = neighbor
            
            if is_dead_end or best_neighbor is None: break

            edge_id = G.get_edge_data(current_node, best_neighbor)['id']
            visited_edges.add(edge_id)
            current_path.append(best_neighbor)
            current_node = best_neighbor
            
            if G.degree(current_node) != 2: break # Stop at intersections

        if len(current_path) > 2: # Min path length
            paths.append(current_path)

    print(f"Found {len(paths)} continuous lanelet paths.")

    # --- 3. Assemble OSM ---
    print("--- Assembling final map... ---")
    for path in paths:
        segments = [G.get_edge_data(path[i], path[i+1])['segment'] for i in range(len(path)-1)]
        centerline = LineString(unary_union(MultiLineString(segments)))
        
        pairs = [G.get_edge_data(path[i], path[i+1])['pair'] for i in range(len(path)-1)]
        if not pairs: continue
        left_id, right_id = max(set(pairs), key=pairs.count)

        left = boundary_gdf.geometry.iloc[left_id]
        right = boundary_gdf.geometry.iloc[right_id]

        # Simplified orientation check
        center_dir = get_line_direction(centerline)
        vec_to_left = np.array(left.interpolate(0.5, normalized=True).coords[0])[:2] - np.array(centerline.interpolate(0.5, normalized=True).coords[0])[:2]
        if np.cross(center_dir, vec_to_left) < 0:
            left, right = right, left # Swap
        
        left_way = osm.add_way(list(left.coords), {"type": "line_thin", "subtype": "solid"})
        right_way = osm.add_way(list(right.coords), {"type": "line_thin", "subtype": "solid"})
        center_way = osm.add_way(list(centerline.coords), {})
        
        osm.add_relation([("way", left_way, "left"), ("way", right_way, "right"), ("way", center_way, "centerline")], {"type": "lanelet", "subtype": "road"})
        
    osm.save_osm(args.output_osm)

if __name__ == "__main__":
    main()
