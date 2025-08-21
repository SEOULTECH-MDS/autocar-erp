import argparse
import geopandas as gpd
import numpy as np
from lxml import etree
from shapely.geometry import LineString, Point, MultiLineString
from shapely.ops import unary_union
from scipy.spatial import KDTree
from collections import defaultdict
import networkx as nx
import math # Import math for validation
from tqdm import tqdm # Import tqdm for progress bar

# (OsmBuilder class remains the same and is omitted for brevity)
class OsmBuilder:
    def __init__(self):
        self.node_id_counter = -1; self.way_id_counter = -1; self.relation_id_counter = -1
        self.nodes = {}; self.ways = []; self.relations = []
    def _get_node_id(self, point):
        coord = (point[0], point[1]);
        if coord not in self.nodes: self.nodes[coord] = self.node_id_counter; self.node_id_counter -= 1
        return self.nodes[coord]
    def add_way(self, points, tags=None):
        if not points: return None
        node_ids = [self._get_node_id(p) for p in points]; way_id = self.way_id_counter; self.way_id_counter -= 1
        self.ways.append({"id": way_id, "nodes": node_ids, "tags": tags or {}}); return way_id
    def add_relation(self, members, tags=None):
        relation_id = self.relation_id_counter; self.relation_id_counter -= 1
        self.relations.append({"id": relation_id, "members": members, "tags": tags or {}}); return relation_id
    def save_osm(self, filename):
        root = etree.Element("osm", version="0.6")
        for coord, node_id in self.nodes.items(): etree.SubElement(root, "node", id=str(node_id), lat=str(coord[1]), lon=str(coord[0]))
        for way_info in self.ways:
            way_elem = etree.SubElement(root, "way", id=str(way_info["id"]))
            for node_id in way_info["nodes"]: etree.SubElement(way_elem, "nd", ref=str(node_id))
            for k, v in way_info["tags"].items(): etree.SubElement(way_elem, "tag", k=k, v=str(v))
        for rel_info in self.relations:
            rel_elem = etree.SubElement(root, "relation", id=str(rel_info["id"]))
            for member_type, ref, role in rel_info["members"]: etree.SubElement(rel_elem, "member", type=member_type, ref=str(ref), role=role)
            for k, v in rel_info["tags"].items(): etree.SubElement(rel_elem, "tag", k=k, v=str(v))
        with open(filename, 'wb') as f: f.write(etree.tostring(root, pretty_print=True, xml_declaration=True, encoding='UTF-8'))
        print(f"Saved OSM file to {filename}")

DEFAULT_LANE_WIDTH_M = 3.5

def get_line_direction(line):
    if line.length < 1e-6: return np.array([0.0, 0.0])
    coords = np.array(line.coords)
    vec = coords[-1, :2] - coords[0, :2]
    norm = np.linalg.norm(vec)
    return vec / norm if norm > 1e-6 else np.array([0.0, 0.0])

def is_line_finite(line):
    """Checks if a LineString contains only finite coordinate values."""
    if not line or line.is_empty or not isinstance(line, LineString):
        return False
    for p in line.coords:
        if not all(math.isfinite(c) for c in p):
            return False
    return True

def main():
    parser = argparse.ArgumentParser(description="Fully automated Lanelet2 converter with CRS reprojection.")
    parser.add_argument("--input_dir", required=True, help="Directory containing the kcity_v3_all shapefiles.")
    parser.add_argument("--output_osm", required=True, help="Path for the final output Lanelet2 OSM file.")
    args = parser.parse_args()

    print("Loading shapefiles...")
    base_path = args.input_dir
    try:
        centerlines_gdf = gpd.read_file(f"{base_path}/link.shp")
        left_right_gdf = gpd.read_file(f"{base_path}/left_right_lines.shp")
        dashed_gdf = gpd.read_file(f"{base_path}/dased_lines.shp")
    except Exception as e:
        print(f"Error loading shapefiles: {e}")
        return

    # --- NEW: CRS REPROJECTION STEP ---
    target_crs = "EPSG:4326"
    print(f"Original CRS: {centerlines_gdf.crs}")
    print(f"Reprojecting all layers to {target_crs}...")
    try:
        centerlines_gdf = centerlines_gdf.to_crs(target_crs)
        left_right_gdf = left_right_gdf.to_crs(target_crs)
        dashed_gdf = dashed_gdf.to_crs(target_crs)
    except Exception as e:
        print(f"Error during CRS reprojection: {e}")
        print("Please ensure the source shapefiles have a valid .prj file.")
        return
    # --- END NEW STEP ---

    all_boundaries_gdf = gpd.pd.concat([left_right_gdf, dashed_gdf], ignore_index=True)
    osm = OsmBuilder()

    print(f"Processing {len(centerlines_gdf)} centerlines from link.shp...")

    # Build KDTree for all boundary points for fast searching
    boundary_kdtree_pts = []
    boundary_kdtree_indices = []
    for i, line in enumerate(all_boundaries_gdf.geometry):
        for point in line.coords:
            boundary_kdtree_pts.append(point[:2])
            boundary_kdtree_indices.append(i)
    boundary_kdtree = KDTree(boundary_kdtree_pts)
    
    # --- WRAP a TQDM progress bar around the main loop ---
    for i, center_row in tqdm(centerlines_gdf.iterrows(), total=len(centerlines_gdf), desc="Processing Centerlines"):
        centerline = center_row.geometry
        if not isinstance(centerline, LineString) or centerline.is_empty: continue
        
        # 1. Candidate Search
        search_corridor = centerline.buffer(DEFAULT_LANE_WIDTH_M * 2)
        candidate_indices = set()
        for point in centerline.coords:
            indices = boundary_kdtree.query_ball_point(point[:2], r=DEFAULT_LANE_WIDTH_M * 2)
            for idx in indices:
                candidate_indices.add(boundary_kdtree_indices[idx])

        if len(candidate_indices) < 2: continue
        
        center_dir = get_line_direction(centerline)
        
        # 2. Separate candidates into left and right
        left_candidates, right_candidates = [], []
        for idx in candidate_indices:
            boundary = all_boundaries_gdf.geometry.iloc[idx]
            # Get a point on the boundary closest to the centerline's midpoint
            p_sample = centerline.interpolate(0.5, normalized=True)
            p_boundary = boundary.interpolate(boundary.project(p_sample))
            vec_to_boundary = np.array(p_boundary.coords[0])[:2] - np.array(p_sample.coords[0])[:2]
            
            if np.cross(center_dir, vec_to_boundary) > 0:
                left_candidates.append(idx)
            else:
                right_candidates.append(idx)
        
        if not left_candidates or not right_candidates: continue
        
        # 3. Score and find the best pair
        best_pair = None
        best_score = float('inf')

        for l_idx in left_candidates:
            for r_idx in right_candidates:
                left_b = all_boundaries_gdf.geometry.iloc[l_idx]
                right_b = all_boundaries_gdf.geometry.iloc[r_idx]
                
                # Scoring logic
                sample_distances_l, sample_distances_r, sample_widths = [], [], []
                for p_center in [centerline.interpolate(d, normalized=True) for d in np.linspace(0.1, 0.9, 10)]:
                    dist_l = p_center.distance(left_b)
                    dist_r = p_center.distance(right_b)
                    
                    if dist_l > DEFAULT_LANE_WIDTH_M or dist_r > DEFAULT_LANE_WIDTH_M: continue
                    
                    sample_distances_l.append(dist_l)
                    sample_distances_r.append(dist_r)
                    sample_widths.append(dist_l + dist_r)
                
                if len(sample_widths) < 5: continue
                
                score = (np.std(sample_distances_l) + np.std(sample_distances_r) + np.std(sample_widths))
                
                if score < best_score:
                    best_score = score
                    best_pair = (l_idx, r_idx)
        
        # 4. Assemble Lanelet
        if best_pair and best_score < 0.5: # Quality Threshold
            left_b = all_boundaries_gdf.geometry.iloc[best_pair[0]]
            right_b = all_boundaries_gdf.geometry.iloc[best_pair[1]]
            
            # --- NEW: VALIDATION AND SANITIZATION STEP ---
            if not (is_line_finite(centerline) and is_line_finite(left_b) and is_line_finite(right_b)):
                print(f"  - WARNING: Skipping lanelet for centerline {i} due to non-finite coordinates.")
                continue
            
            # Final orientation check
            left_way = osm.add_way(list(left_b.coords), {"type": "line_thin"})
            right_way = osm.add_way(list(right_b.coords), {"type": "line_thin"})
            center_way = osm.add_way(list(centerline.coords), {})
            osm.add_relation([("way", left_way, "left"), ("way", right_way, "right"), ("way", center_way, "centerline")], {"type": "lanelet", "subtype": "road"})
    
    print(f"Assembled {len(osm.relations)} valid lanelets.")
    osm.save_osm(args.output_osm)

if __name__ == "__main__":
    main()
