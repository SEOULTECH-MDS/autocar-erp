import numpy as np
import shapely.ops as ops
from lxml import etree
from scipy.spatial import KDTree
from shapely.geometry import LineString, Point, Polygon
from scipy.interpolate import splprep, splev

from utils.osm_builder import OsmBuilder
from utils.projector import Projector
from utils.map_config import MapConfig

class BoundaryFirstConverter:
    def __init__(self, input_dir: str, output_osm: str, output_projector_yaml: str, output_map_config_yaml: str):
        self.input_dir = input_dir
        self.output_osm = output_osm
        self.output_projector_yaml = output_projector_yaml
        self.output_map_config_yaml = output_map_config_yaml
        self.gdf = {}
        self.osm = OsmBuilder() # Initialize OsmBuilder

    def load_layers(self):
        # ... (Implementation for loading boundary pairs from GeoJSON)
        pass

    def generate_centerlines(self):
        # ... (Implementation for creating centerlines from boundary pairs)
        if not hasattr(self, 'lanelet_pairs') or not self.lanelet_pairs:
            print("No lanelet pairs to generate centerlines from.")
            return

        self.new_centerlines = []
        for left, right in self.lanelet_pairs:
            # Generate centerline by finding midpoints along the boundaries
            shorter, longer = (left, right) if left.length < right.length else (right, left)
            
            points = []
            for i in np.linspace(0, 1, int(shorter.length / 2)): # sample every 2 meters
                p_shorter = shorter.interpolate(i, normalized=True)
                p_longer = longer.interpolate(longer.project(p_shorter))
                mid_x = (p_shorter.x + p_longer.x) / 2
                mid_y = (p_shorter.y + p_longer.y) / 2
                points.append((mid_x, mid_y))

            if len(points) >= 2:
                # Smooth the centerline
                if len(points) > 3:
                    try:
                        tck, u = splprep([np.array(points)[:,0], np.array(points)[:,1]], s=1.0, k=3)
                        x_new, y_new = splev(np.linspace(0, 1, 100), tck)
                        self.new_centerlines.append(LineString(zip(x_new, y_new)))
                    except Exception:
                        self.new_centerlines.append(LineString(points)) # Fallback
                else:
                    self.new_centerlines.append(LineString(points))
        
        print(f"Generated {len(self.new_centerlines)} new centerlines.")
    
    def refine_intersection_fillets(self):
        # (Implementation for using sidewalk corners to create smooth turns)
        pass

    def run(self):
        self.load_layers()
        self.generate_centerlines()
        # 5. Refine intersections
        # self.refine_intersection_fillets() # Disabled for now
        # 6. Build and save OSM
        self.build_and_save_osm()
        print("Conversion complete.")

    def build_and_save_osm(self):
        if not hasattr(self, 'lanelet_pairs') or not self.lanelet_pairs:
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
        
        self.osm.save_osm(self.output_osm)

def main():
    # ... existing code ...
    pass
