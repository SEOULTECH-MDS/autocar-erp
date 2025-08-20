import geopandas as gpd
from shapely.geometry import Point, LineString
from scipy.spatial import KDTree
from collections import defaultdict, Counter
import numpy as np
import argparse

DEFAULT_LANE_WIDTH_M = 3.5
INTERSECTION_BUFFER_M = 5.0 # Radius to define intersection areas

def find_intersection_hubs(boundary_gdf):
    """Finds points where more than 2 boundary lines meet."""
    print("Finding intersection hubs...")
    endpoints = []
    for line in boundary_gdf.geometry:
        if line and not line.is_empty:
            endpoints.append(line.coords[0])
            endpoints.append(line.coords[-1])
    
    hub_points = [point for point, count in Counter(endpoints).items() if count > 2]
    
    if not hub_points:
        print("No significant hubs found.")
        return None
        
    hub_gdf = gpd.GeoDataFrame(geometry=[Point(p) for p in hub_points], crs=boundary_gdf.crs)
    intersection_polygons = hub_gdf.buffer(INTERSECTION_BUFFER_M)
    print(f"Identified {len(hub_points)} intersection hubs.")
    return gpd.GeoDataFrame(geometry=intersection_polygons, crs=boundary_gdf.crs)

def process_easy_segments(easy_segments_gdf, boundary_gdf):
    """Processes non-intersection segments to automatically find lanelets."""
    print(f"Automatically processing {len(easy_segments_gdf)} 'easy' segments...")
    if easy_segments_gdf.empty:
        return []

    # Build KDTree from all boundary vertices for fast searching
    boundary_points = []
    boundary_line_indices = []
    for i, line in enumerate(boundary_gdf.geometry):
        for point in line.coords:
            boundary_points.append(point[:2])
            boundary_line_indices.append(i)
    kdtree = KDTree(boundary_points)

    automated_lanelets = []
    
    for _, row in easy_segments_gdf.iterrows():
        line = row.geometry
        sample_points = [line.interpolate(d, normalized=True) for d in np.linspace(0.2, 0.8, 5)]
        consistent_pair_votes = defaultdict(int)

        for p in sample_points:
            dists, indices = kdtree.query(p.coords[0][:2], k=20)
            closest_lines = {}
            for dist, idx in zip(dists, indices):
                line_idx = boundary_line_indices[idx]
                if line_idx not in closest_lines or dist < closest_lines[line_idx]:
                    closest_lines[line_idx] = dist
            
            if len(closest_lines) < 2: continue
            sorted_closest = sorted(closest_lines.items(), key=lambda item: item[1])
            idx1, dist1 = sorted_closest[0]
            idx2, dist2 = sorted_closest[1]
            
            if (dist1 + dist2) < DEFAULT_LANE_WIDTH_M * 1.8 and (0.5 < dist1 / (dist2 + 1e-9) < 2.0):
                pair = tuple(sorted((idx1, idx2)))
                consistent_pair_votes[pair] += 1
        
        if not consistent_pair_votes: continue
        best_pair = max(consistent_pair_votes, key=consistent_pair_votes.get)
        if consistent_pair_votes[best_pair] >= 3:
             automated_lanelets.append({
                'geometry': line,
                'left_id': best_pair[0], # Placeholder, will be corrected later
                'right_id': best_pair[1]
            })
            
    print(f"Successfully automated {len(automated_lanelets)} lanelets.")
    return automated_lanelets

def main():
    parser = argparse.ArgumentParser(description="Intelligently automate lanelet creation, separating easy sections from hard intersections.")
    parser.add_argument("--boundaries", required=True, help="Path to the boundary network GeoJSON.")
    parser.add_argument("--segments", required=True, help="Path to the exploded Voronoi segments GeoJSON.")
    parser.add_argument("--auto_output", required=True, help="Output file for automatically generated lanelets.")
    parser.add_argument("--manual_output", required=True, help="Output file for segments requiring manual review.")
    args = parser.parse_args()

    boundary_gdf = gpd.read_file(args.boundaries)
    segments_gdf = gpd.read_file(args.segments)

    intersection_hubs_gdf = find_intersection_hubs(boundary_gdf)
    
    if intersection_hubs_gdf is not None and not intersection_hubs_gdf.empty:
        # Separate easy and hard segments
        hard_indices = gpd.sjoin(segments_gdf, intersection_hubs_gdf, how="inner", predicate="intersects").index.unique()
        easy_segments_gdf = segments_gdf.drop(hard_indices)
        manual_review_gdf = segments_gdf.loc[hard_indices]
    else:
        easy_segments_gdf = segments_gdf
        manual_review_gdf = gpd.GeoDataFrame(geometry=[], crs=segments_gdf.crs)
        
    print(f"Separated segments: {len(easy_segments_gdf)} for automation, {len(manual_review_gdf)} for manual review.")

    # Process easy segments
    automated_lanelets_data = process_easy_segments(easy_segments_gdf, boundary_gdf)
    if automated_lanelets_data:
        auto_gdf = gpd.GeoDataFrame(automated_lanelets_data, crs=segments_gdf.crs)
        auto_gdf.to_file(args.auto_output, driver='GeoJSON')
        print(f"Saved automated lanelets to {args.auto_output}")
    else:
        print("No lanelets were automated.")

    # Save segments needing manual review
    manual_review_gdf.to_file(args.manual_output, driver='GeoJSON')
    print(f"Saved segments requiring manual review to {args.manual_output}")

if __name__ == "__main__":
    main()
