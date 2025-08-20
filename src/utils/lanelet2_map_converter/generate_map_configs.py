
import xml.etree.ElementTree as ET
import os
import yaml
import mgrs

def calculate_map_center(osm_path):
    """Calculates the center latitude and longitude of an OSM map."""
    try:
        tree = ET.parse(osm_path)
        root = tree.getroot()
        
        lats, lons = [], []
        for node in root.findall('node'):
            lat, lon = node.get('lat'), node.get('lon')
            if lat and lon:
                lats.append(float(lat))
                lons.append(float(lon))
                
        if not lats:
            return None, None
            
        return sum(lats) / len(lats), sum(lons) / len(lons)
        
    except (ET.ParseError, IOError) as e:
        print(f"Error processing OSM file {osm_path}: {e}")
        return None, None

def generate_projector_info_from_mirae(center_lat, center_lon, output_path):
    """Generates map_projector_info.yaml based on the mirae_map format."""
    m = mgrs.MGRS()
    mgrs_code = m.toMGRS(center_lat, center_lon)
    # Extract the grid zone from the full MGRS code (e.g., "52SCG" from "52SCG...")
    grid_zone = mgrs_code[:5]

    projector_data = {
        'projector_type': 'MGRS',
        'vertical_datum': 'WGS84',
        'mgrs_grid': grid_zone 
    }
    
    with open(output_path, 'w') as f:
        yaml.dump(projector_data, f, default_flow_style=False)
    print(f"Generated MGRS-style map_projector_info.yaml at {output_path}")

def generate_map_config_from_mirae(center_lat, center_lon, output_path):
    """Generates map_config.yaml based on the mirae_map format."""
    config_data = {
        '/**': {
            'ros__parameters': {
                'map_origin': {
                    'latitude': center_lat,
                    'longitude': center_lon,
                    'elevation': 0.0,
                    'roll': 0.0,
                    'pitch': 0.0,
                    'yaw': 0.0
                }
            }
        }
    }
    with open(output_path, 'w') as f:
        # Use a custom dumper to avoid aliases for the '**' key
        yaml.dump(config_data, f, default_flow_style=False, Dumper=yaml.SafeDumper)
    print(f"Generated origin-style map_config.yaml at {output_path}")

if __name__ == '__main__':
    kcity_map_dir = '/home/hmmdyn/autocar-erp/src/core/localization/localization_core/data/kcity_v1'
    osm_file = os.path.join(kcity_map_dir, 'lanelet2_map.osm')
    
    if not os.path.exists(osm_file):
        print(f"Error: Input OSM file not found at {osm_file}")
    else:
        lat, lon = calculate_map_center(osm_file)
        
        if lat is not None and lon is not None:
            print(f"Calculated map center: Latitude={lat}, Longitude={lon}")
            
            # Generate projector info
            projector_info_file = os.path.join(kcity_map_dir, 'map_projector_info.yaml')
            generate_projector_info_from_mirae(lat, lon, projector_info_file)
            
            # Generate map config
            map_config_file = os.path.join(kcity_map_dir, 'map_config.yaml')
            generate_map_config_from_mirae(lat, lon, map_config_file)
