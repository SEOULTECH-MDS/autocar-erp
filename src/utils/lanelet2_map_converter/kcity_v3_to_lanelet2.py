import argparse
import itertools
import json
import math
import os
from collections import defaultdict
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import geopandas as gpd
import numpy as np
import shapely.ops as ops
from lxml import etree
from scipy.spatial import KDTree, distance
from shapely.geometry import LineString, Point, Polygon, MultiLineString
from shapely.strtree import STRtree

# Add necessary imports for centerline generation
from scipy.interpolate import splprep, splev

try:
    import mgrs
except Exception:
    mgrs = None


# --- Configuration ---
DEFAULT_LANE_WIDTH_M = 3.5
DEFAULT_LINE_WIDTH_TAG_M = 0.200
DEFAULT_LOCATION = "urban"
DEFAULT_ONE_WAY = "yes"
DEFAULT_SPEED_LIMIT = 30

COORD_ROUND_DECIMALS = 7
BOUNDARY_CONNECT_THRESH_M = 0.5


# --- Data Models ---
@dataclass
class Way:
    node_refs: List[str]
    tags: Dict[str, str]

@dataclass
class Relation:
    members: List[Tuple[str, str, str]]
    tags: Dict[str, str]

# ... (OsmBuilder and other utility classes will be added here) ...

# --- Utility Classes (OsmBuilder, etc.) ---
# (This would be where the OsmBuilder and other helper classes from the previous version go)
class IdGenerator:
    def __init__(self) -> None:
        self.node_id = 1
        self.way_id = 1
        self.rel_id = 1

    def next_node(self) -> str:
        nid = str(self.node_id)
        self.node_id += 1
        return nid

    def next_way(self) -> str:
        wid = str(self.way_id)
        self.way_id += 1
        return wid

    def next_relation(self) -> str:
        rid = str(self.rel_id)
        self.rel_id += 1
        return rid

class OsmBuilder:
    def __init__(self) -> None:
        self.nodes: Dict[str, Tuple[float, float]] = {}
        self.node_coord_to_id: Dict[Tuple[float, float], str] = {}
        self.ways: Dict[str, Way] = {}
        self.relations: Dict[str, Relation] = {}
        self.ids = IdGenerator()

    def _round_coord(self, lon: float, lat: float) -> Tuple[float, float]:
        return (round(lon, COORD_ROUND_DECIMALS), round(lat, COORD_ROUND_DECIMALS))

    def add_node(self, lon: float, lat: float) -> str:
        coord = self._round_coord(lon, lat)
        if coord in self.node_coord_to_id:
            return self.node_coord_to_id[coord]
        nid = self.ids.next_node()
        self.nodes[nid] = coord
        self.node_coord_to_id[coord] = nid
        return nid

    def add_way(self, coords: List[Tuple[float, float]], tags: Optional[Dict[str, str]] = None) -> Optional[str]:
        if not coords or len(coords) < 2:
            return None
        node_refs = [self.add_node(lon, lat) for lon, lat in coords]
        wid = self.ids.next_way()
        self.ways[wid] = Way(node_refs=node_refs, tags=tags or {})
        return wid

    def add_relation(self, members: List[Tuple[str, str, str]], tags: Optional[Dict[str, str]] = None) -> str:
        rid = self.ids.next_relation()
        self.relations[rid] = Relation(members=members, tags=tags or {})
        return rid

    def save_osm(self, output_path: str) -> None:
        osm_root = etree.Element("osm", version="0.6", generator="BoundaryFirstConverter")
        for node_id, (lon, lat) in self.nodes.items():
            etree.SubElement(osm_root, "node", id=node_id, lat=str(lat), lon=str(lon))
        for way_id, way in self.ways.items():
            way_elem = etree.SubElement(osm_root, "way", id=way_id)
            for nref in way.node_refs:
                etree.SubElement(way_elem, "nd", ref=nref)
            for k, v in way.tags.items():
                etree.SubElement(way_elem, "tag", k=str(k), v=str(v))
        for rel_id, rel in self.relations.items():
            rel_elem = etree.SubElement(osm_root, "relation", id=rel_id)
            for m_type, m_ref, m_role in rel.members:
                etree.SubElement(rel_elem, "member", type=m_type, ref=m_ref, role=m_role)
            for k, v in rel.tags.items():
                etree.SubElement(rel_elem, "tag", k=str(k), v=str(v))
        xml_str = etree.tostring(osm_root, pretty_print=True, xml_declaration=True, encoding="UTF-8")
        with open(output_path, "wb") as f:
            f.write(xml_str)

def _get_line_direction(line):
    if line.length < 1e-6:
        return np.array([0, 0])
    coords = np.array(line.coords)
    start = coords[0, :2] # Explicitly take only X, Y
    end = coords[-1, :2]  # Explicitly take only X, Y
    vec = end - start
    norm = np.linalg.norm(vec)
    if norm < 1e-6:
        return np.array([0, 0])
    return vec / norm

class BoundaryFirstConverter:
    def __init__(self, input_dir: str, output_osm: str, output_projector_yaml: str, output_map_config_yaml: str):
        self.input_dir = input_dir
        self.output_osm = output_osm
        # ... (other initializations) ...
        self.gdf = {}
        self.osm = OsmBuilder() # Initialize OsmBuilder
        self.lanelet_pairs: List[Tuple[LineString, LineString]] = [] # New attribute for lanelet pairs

    def load_layers(self):
        # (Implementation for loading shapefiles)
        shapefile_names = [
            "left_right_lines", "dased_lines", "dashed_lines", 
            "link", "sidewalk_area", "stopline", "traffic_location"
        ]
        for name in shapefile_names:
            path = os.path.join(self.input_dir, f"{name}.shp")
            if os.path.exists(path):
                try:
                    gdf = gpd.read_file(path)
                    if gdf.crs and gdf.crs.to_epsg() != 4326:
                        gdf = gdf.to_crs("EPSG:4326")
                    self.gdf[name] = gdf[~gdf.geometry.is_empty & gdf.geometry.is_valid].copy()
                except Exception as e:
                    print(f"Warning: Could not load or reproject {name}.shp: {e}")

    def build_boundary_network(self):
        boundary_gdfs = []
        if "left_right_lines" in self.gdf:
            boundary_gdfs.append(self.gdf["left_right_lines"])
        
        dashed_name = "dashed_lines" if "dashed_lines" in self.gdf else "dased_lines"
        if dashed_name in self.gdf:
            boundary_gdfs.append(self.gdf[dashed_name])

        if not boundary_gdfs:
            print("Error: No boundary line shapefiles found.")
            return

        all_boundaries = gpd.pd.concat(boundary_gdfs, ignore_index=True)
        
        lines = []
        for geom in all_boundaries.geometry:
            if geom.geom_type == 'LineString':
                lines.append(geom)
            elif geom.geom_type == 'MultiLineString':
                lines.extend(list(geom.geoms))

        # Use shapely.ops.linemerge to connect touching lines
        merged = ops.linemerge(lines)
        
        final_lines = []
        if merged.geom_type == 'LineString':
            final_lines.append(merged)
        elif merged.geom_type == 'MultiLineString':
            final_lines.extend(list(merged.geoms))

        self.boundary_network = gpd.GeoDataFrame(geometry=final_lines, crs="EPSG:4326")
        print(f"Built boundary network with {len(self.boundary_network)} continuous lines.")

        # --- DEBUG: Save boundary network to GeoJSON ---
        try:
            self.boundary_network.to_file("debug_boundary_network.geojson", driver='GeoJSON')
            print("--- Saved debug_boundary_network.geojson ---")
        except Exception as e:
            print(f"Could not save debug file: {e}")
        # --- END DEBUG ---

    def identify_lanelet_pairs(self):
        # This method will be replaced by centerline generation via Voronoi
        pass

    def generate_centerlines_from_voronoi(self):
        if not hasattr(self, 'boundary_network') or self.boundary_network.empty:
            return

        # Create a single MultiLineString for Voronoi input
        all_boundaries = MultiLineString(self.boundary_network.geometry.tolist())
        
        # Create a slightly larger bounding box to avoid edge effects
        bounds = all_boundaries.bounds
        buffer = 10 # meters in projected coordinates, approx
        envelope = Polygon([(bounds[0]-buffer, bounds[1]-buffer), 
                            (bounds[0]-buffer, bounds[3]+buffer), 
                            (bounds[2]+buffer, bounds[3]+buffer), 
                            (bounds[2]+buffer, bounds[1]-buffer)])

        # Generate Voronoi diagram clipped by the envelope
        voronoi = ops.voronoi_diagram(all_boundaries, envelope=envelope)
        try:
            # FIX 2: If voronoi returns Polygons, extract their boundaries as LineStrings.
            voronoi_geoms = list(voronoi.geoms)
            if voronoi_geoms and isinstance(voronoi_geoms[0], Polygon):
                voronoi_lines = [geom.boundary for geom in voronoi_geoms]
            else:
                voronoi_lines = [geom for geom in voronoi_geoms if isinstance(geom, LineString)]

            gpd.GeoDataFrame(geometry=voronoi_lines, crs="EPSG:4326").to_file("debug_raw_voronoi_lines_only.geojson", driver='GeoJSON')
            print("--- Saved debug_raw_voronoi_lines_only.geojson ---")
        except Exception as e:
            print(f"Could not save debug file: {e}")
        
        # --- DEBUG ---
        # Save the original raw voronoi diagram for comparison if needed
        try:
            gpd.GeoDataFrame(geometry=list(voronoi.geoms), crs="EPSG:4326").to_file("debug_raw_voronoi.geojson", driver='GeoJSON')
            print("--- Saved debug_raw_voronoi.geojson ---")
        except Exception as e:
            print(f"Could not save debug file: {e}")
        # --- END DEBUG ---
        
        # --- Final Approach: KDTree on boundary vertices ---
        self.new_centerlines = []
        self.lanelet_pairs = []

        # 1. Build KDTree from all boundary vertices
        boundary_points = []
        boundary_line_indices = [] # Store which line each point belongs to
        for i, line in enumerate(self.boundary_network.geometry):
            for point in line.coords:
                boundary_points.append(point[:2])
                boundary_line_indices.append(i)
        
        if not boundary_points:
            return

        kdtree = KDTree(boundary_points)

        # 2. Filter Voronoi edges
        for line in voronoi.geoms:
            if line.geom_type != 'LineString' or line.length < DEFAULT_LANE_WIDTH_M:
                continue

            sample_points = [line.interpolate(d, normalized=True) for d in np.linspace(0.2, 0.8, 5)]
            consistent_pair_votes = defaultdict(int)

            for p in sample_points:
                # Find N nearest vertices
                dists, indices = kdtree.query(p.coords[0][:2], k=20)
                
                # Find the two unique boundary lines these vertices belong to
                closest_lines = {} # line_idx -> min_dist
                for dist, idx in zip(dists, indices):
                    line_idx = boundary_line_indices[idx]
                    if line_idx not in closest_lines or dist < closest_lines[line_idx]:
                        closest_lines[line_idx] = dist
                
                if len(closest_lines) < 2:
                    continue

                sorted_closest = sorted(closest_lines.items(), key=lambda item: item[1])
                idx1, dist1 = sorted_closest[0]
                idx2, dist2 = sorted_closest[1]
                
                # Equidistance and proximity check
                if (dist1 + dist2) < DEFAULT_LANE_WIDTH_M * 1.8 and (0.6 < dist1 / (dist2 + 1e-9) < 1.6):
                    pair = tuple(sorted((idx1, idx2)))
                    consistent_pair_votes[pair] += 1
            
            # Check for consistency
            if not consistent_pair_votes:
                continue
            
            best_pair = max(consistent_pair_votes, key=consistent_pair_votes.get)
            if consistent_pair_votes[best_pair] >= 3: # At least 3/5 samples agree
                self.new_centerlines.append(line)
                b1 = self.boundary_network.geometry.iloc[best_pair[0]]
                b2 = self.boundary_network.geometry.iloc[best_pair[1]]
                self.lanelet_pairs.append((b1, b2))

        print(f"Generated {len(self.new_centerlines)} refined centerlines using KDTree.")


    def generate_centerlines(self):
        # This method is now fully replaced
        pass

    def refine_intersection_fillets(self):
        # (Implementation for using sidewalk corners to create smooth turns)
        pass

    def run(self):
        print("Starting Boundary-First Reconstruction...")
        # 1. Load data
        self.load_layers()
        # 2. Build continuous boundary network
        self.build_boundary_network()
        # 3. Find lanelet boundary pairs
        # self.identify_lanelet_pairs() # Deprecated
        # 4. Generate new centerlines
        self.generate_centerlines_from_voronoi()
        # self.generate_centerlines() # Deprecated
        # 5. Refine intersections
        self.refine_intersection_fillets()
        # 6. Build and save OSM
        self.build_and_save_osm()
        print("Conversion complete.")

    def build_and_save_osm(self):
        if not hasattr(self, 'lanelet_pairs') or not self.lanelet_pairs:
            print("No lanelets to build.")
            return

        for i, (left, right) in enumerate(self.lanelet_pairs):
            if i >= len(self.new_centerlines):
                continue
            
            centerline = self.new_centerlines[i]

            left_way_id = self.osm.add_way(list(left.coords), {"type": "line_thin", "subtype": "solid"})
            right_way_id = self.osm.add_way(list(right.coords), {"type": "line_thin", "subtype": "solid"})
            centerline_way_id = self.osm.add_way(list(centerline.coords), {})

            if left_way_id and right_way_id and centerline_way_id:
                self.osm.add_relation(
                    members=[
                        ("way", left_way_id, "left"),
                        ("way", right_way_id, "right"),
                        ("way", centerline_way_id, "centerline"),
                    ],
                    tags={"type": "lanelet", "subtype": "road"}
                )
        
        print(f"Building OSM with {len(self.osm.relations)} lanelets.")
        self.osm.save_osm(self.output_osm)

def main():
    parser = argparse.ArgumentParser(description="Convert KCity V3 shapefiles to Lanelet2 OSM using Boundary-First Reconstruction.")
    parser.add_argument("--input_dir", required=True)
    parser.add_argument("--output_osm", required=True, default="lanelet2_map_boundary_first.osm")
    parser.add_argument("--output_projector_yaml", required=False, default="map_projector_info.yaml")
    parser.add_argument("--output_map_config_yaml", required=False, default="map_config.yaml")
    args = parser.parse_args()

    converter = BoundaryFirstConverter(
        args.input_dir,
        args.output_osm,
        args.output_projector_yaml,
        args.output_map_config_yaml
    )
    converter.run()

if __name__ == "__main__":
    main()
