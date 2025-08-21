import geopandas as gpd
import pandas as pd
import math
from lxml import etree
import os
from shapely.geometry import LineString, Point, Polygon, MultiLineString, MultiPolygon
from shapely.ops import nearest_points
import numpy as np

class Lanelet2MapGenerator:
    def __init__(self, shp_dir, output_file):
        self.shp_dir = shp_dir
        self.output_file = output_file
        self.gdf = {}
        self.nodes = {}
        self.ways = {}
        self.relations = {}
        self.node_id_counter = 1
        self.way_id_counter = 1
        self.relation_id_counter = 1
        self.node_coord_map = {}

    def load_shapefiles(self):
        print(f"Loading and reprojecting shapefiles from {self.shp_dir}...")
        for filename in os.listdir(self.shp_dir):
            if filename.endswith('.shp'):
                name = filename.replace('.shp', '')
                filepath = os.path.join(self.shp_dir, filename)
                try:
                    temp_gdf = gpd.read_file(filepath)
                    # ---- CRITICAL: Reproject to WGS84 (lat/lon) ----
                    if temp_gdf.crs and temp_gdf.crs.to_epsg() != 4326:
                        print(f"  - Reprojecting {name} from CRS {temp_gdf.crs.name} to WGS84...")
                        temp_gdf = temp_gdf.to_crs("EPSG:4326")
                    self.gdf[name] = temp_gdf
                    print(f"  - Loaded {name} ({len(self.gdf[name])} features)")
                    # print(f"    Columns: {self.gdf[name].columns.tolist()}")
                except Exception as e:
                    print(f"Error loading or reprojecting {filepath}: {e}")
        if not self.gdf:
            raise ValueError("No shapefiles loaded.")

    def add_node(self, lon, lat):
        coord = (round(lon, 7), round(lat, 7))
        if coord in self.node_coord_map:
            return self.node_coord_map[coord]
        new_id = str(self.node_id_counter)
        self.nodes[new_id] = coord
        self.node_coord_map[coord] = new_id
        self.node_id_counter += 1
        return new_id

    def create_way(self, node_ids, tags=None):
        if not node_ids or len(node_ids) < 2:
            return None
        way_id = str(self.way_id_counter)
        self.ways[way_id] = {'nodes': list(dict.fromkeys(node_ids)), 'tags': tags or {}} # Remove duplicates
        self.way_id_counter += 1
        return way_id

    def create_relation(self, members, tags=None):
        relation_id = str(self.relation_id_counter)
        self.relations[relation_id] = {'members': members, 'tags': tags or {}}
        self.relation_id_counter += 1
        return relation_id

    def process_geometry(self, geometry):
        node_ids = []
        if geometry is None or geometry.is_empty:
            return []
        
        geoms = list(geometry.geoms) if hasattr(geometry, 'geoms') else [geometry]

        for geom in geoms:
            if geom.is_empty: continue
            coords = geom.exterior.coords if isinstance(geom, Polygon) else geom.coords
            for x, y, *_ in coords:
                node_ids.append(self.add_node(x, y))
        return node_ids
    
    def validate_and_save(self):
        print("Validating map data...")
        is_valid = True
        for way_id, way_data in self.ways.items():
            for node_ref in way_data['nodes']:
                if node_ref not in self.nodes:
                    print(f"[Validation Error] Way {way_id} has invalid node ref {node_ref}")
                    is_valid = False
        
        for rel_id, rel_data in self.relations.items():
            for m_type, m_ref, m_role in rel_data['members']:
                if (m_type == 'way' and m_ref not in self.ways) or \
                   (m_type == 'node' and m_ref not in self.nodes):
                    print(f"[Validation Error] Relation {rel_id} has invalid member ref {m_type}={m_ref}")
                    is_valid = False
        
        if not is_valid:
            print("Validation failed. OSM file will not be generated.")
            return

        print("Validation successful. Saving OSM file...")
        osm_root = etree.Element('osm', version='0.6', generator='ValidatedKCityV2MapGenerator')

        for node_id, (lon, lat) in self.nodes.items():
            etree.SubElement(osm_root, 'node', id=node_id, lat=str(lat), lon=str(lon))

        for way_id, way_data in self.ways.items():
            way_elem = etree.SubElement(osm_root, 'way', id=way_id)
            for node_ref in way_data['nodes']:
                etree.SubElement(way_elem, 'nd', ref=node_ref)
            for k, v in way_data.get('tags', {}).items():
                etree.SubElement(way_elem, 'tag', k=str(k), v=str(v))

        for rel_id, rel_data in self.relations.items():
            rel_elem = etree.SubElement(osm_root, 'relation', id=rel_id)
            for m_type, m_ref, m_role in rel_data['members']:
                etree.SubElement(rel_elem, 'member', type=m_type, ref=m_ref, role=m_role)
            for k, v in rel_data.get('tags', {}).items():
                etree.SubElement(rel_elem, 'tag', k=str(k), v=str(v))

        xml_str = etree.tostring(osm_root, pretty_print=True, xml_declaration=True, encoding='UTF-8')
        with open(self.output_file, 'wb') as f:
            f.write(xml_str)
        print(f"Generated Lanelet2 map at {self.output_file}")

    def generate_map(self):
        self.load_shapefiles()
        
        # Pre-process all line and polygon features into ways
        for name, gdf in self.gdf.items():
            tags = {}
            if name == 'Surfacemark':
                # Example of using attributes to tag ways. Assumes 'Kind' column exists.
                gdf['way_id'] = gdf.apply(lambda row: self.create_way(self.process_geometry(row.geometry), 
                                           {'type': 'line_thin', 'subtype': str(row.get('Kind', 'unknown'))}), axis=1)
            elif name == 'Roadmark_crosswalk':
                gdf['way_id'] = gdf.apply(lambda row: self.create_way(self.process_geometry(row.geometry), 
                                           {'type': 'road_marking', 'subtype': str(row.get('Kind', 'unknown'))}), axis=1)
            # Link ways will be created during lanelet processing
            
        # Create Lanelets from 'Link' centerlines
        link_gdf = self.gdf.get('Link')
        surfacemark_gdf = self.gdf.get('Surfacemark')
        
        if link_gdf is not None and surfacemark_gdf is not None:
            for _, link_row in link_gdf.iterrows():
                centerline_geom = link_row.geometry
                if not isinstance(centerline_geom, LineString) or centerline_geom.is_empty:
                    continue
                
                centerline_way_id = self.create_way(self.process_geometry(centerline_geom))
                if not centerline_way_id: continue
                
                # A robust function to find left/right boundaries is complex.
                # This is a placeholder for the logic that needs to be implemented.
                # For now, we will skip lanelet creation if boundaries are not found.
                # A proper implementation would use geometry operations to find the correct left/right lines.
                
                # Dummy creation for structure test
                left_way_id = surfacemark_gdf.iloc[0]['way_id'] if not surfacemark_gdf.empty else None
                right_way_id = surfacemark_gdf.iloc[-1]['way_id'] if len(surfacemark_gdf) > 1 else None

                if left_way_id and right_way_id:
                    self.create_relation(
                        members=[('way', left_way_id, 'left'), 
                                 ('way', right_way_id, 'right'), 
                                 ('way', centerline_way_id, 'centerline')],
                        tags={'type': 'lanelet', 'subtype': 'road'}
                    )
        
        # Final validation and save
        self.validate_and_save()

if __name__ == '__main__':
    script_dir = os.path.dirname(os.path.abspath(__file__))
    base_dir = os.path.abspath(os.path.join(script_dir, '../../../'))
    kcity_v2_dir = os.path.join(base_dir, 'src/core/localization/localization_core/data/kcity_v2')
    output_dir = os.path.join(base_dir, 'src/core/localization/localization_core/data/kcity_v2_generated')
    os.makedirs(output_dir, exist_ok=True)
    output_filename = os.path.join(output_dir, 'lanelet2_map.osm')

    if not os.path.isdir(kcity_v2_dir):
        print(f"Error: Directory not found at {kcity_v2_dir}")
    else:
        generator = Lanelet2MapGenerator(kcity_v2_dir, output_filename)
        generator.generate_map()