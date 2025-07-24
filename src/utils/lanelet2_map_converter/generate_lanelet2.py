import math
import xml.etree.ElementTree as ET
from xml.dom import minidom
import mgrs # For MGRS code generation

# --- Configuration ---
LANE_WIDTH = 3.5
LINE_WIDTH_TAG = '0.200'

def load_osm_data(file_paths):
    nodes = {}
    ways = []
    for file_path in file_paths:
        try:
            tree = ET.parse(file_path)
            root = tree.getroot()
            for node_elem in root.findall('node'):
                node_id = node_elem.get('id')
                if node_id not in nodes:
                    lat = node_elem.get('lat')
                    lon = node_elem.get('lon')
                    if lat and lon:
                        nodes[node_id] = {'lat': float(lat), 'lon': float(lon)}
            for way_elem in root.findall('way'):
                node_refs = [nd.get('ref') for nd in way_elem.findall('nd') if nd.get('ref')]
                if node_refs:
                    ways.append(node_refs)
        except (ET.ParseError, IOError) as e:
            print(f"Warning: Could not process {file_path}. Error: {e}")
    return nodes, ways

def create_trajectories(ways):
    # This function is simplified for clarity, assuming a single continuous loop or path
    # A more robust implementation would handle multiple disjoint graphs
    if not ways:
        return []
    
    trajectories = []
    while ways:
        current_chain = ways.pop(0)
        # Extend the chain
        i = 0
        while i < len(ways):
            if ways[i][0] == current_chain[-1]:
                current_chain.extend(ways.pop(i)[1:])
                i = -1 # Restart scan
            elif ways[i][-1] == current_chain[0]:
                current_chain = ways.pop(i)[:-1] + current_chain
                i = -1 # Restart scan
            i += 1
        trajectories.append(current_chain)
    return trajectories

def generate_lanelet2_map(osm_files, output_file):
    nodes, ways = load_osm_data(osm_files)
    centerline_trajectories = create_trajectories(ways)
    
    m = mgrs.MGRS()
    osm_root = ET.Element('osm', version='0.6', generator='CustomLanelet2Generator')
    node_id_counter = 1
    way_id_counter = 1
    relation_id_counter = 1

    node_map = {} # (lat, lon) -> new_id

    def add_node(lat, lon, original_id=None):
        nonlocal node_id_counter
        # Use original_id to key into map if available
        key = original_id if original_id else (lat, lon)
        if key in node_map:
            return node_map[key]

        new_id = str(node_id_counter)
        node_elem = ET.SubElement(osm_root, 'node', id=new_id, visible='true', version='1', lat=str(lat), lon=str(lon))
        
        # Add MGRS tag
        mgrs_code = m.toMGRS(lat, lon)
        ET.SubElement(node_elem, 'tag', k='mgrs_code', v=str(mgrs_code))
        
        node_map[key] = new_id
        node_id_counter += 1
        return new_id

    # Pre-populate map with original nodes
    for node_id, data in nodes.items():
        add_node(data['lat'], data['lon'], original_id=node_id)

    for trajectory in centerline_trajectories:
        left_boundary_nodes, right_boundary_nodes, centerline_way_nodes = [], [], []

        for node_id in trajectory:
            centerline_way_nodes.append(node_map[node_id])

        for i in range(len(trajectory)):
            p_curr = nodes[trajectory[i]]
            
            if i < len(trajectory) - 1:
                p_next = nodes[trajectory[i+1]]
                angle = math.atan2(p_next['lat'] - p_curr['lat'], p_next['lon'] - p_curr['lon'])
            else:
                p_prev = nodes[trajectory[i-1]]
                angle = math.atan2(p_curr['lat'] - p_prev['lat'], p_curr['lon'] - p_prev['lon'])

            norm_angle = angle + math.pi / 2
            # Note: This is a simplification. Real-world meters conversion from lat/lon diff is complex.
            # This works for small distances, assuming a ~111,111 meter per degree ratio.
            m_per_deg = 111111
            dx = math.cos(norm_angle) * (LANE_WIDTH / 2) / (m_per_deg * math.cos(math.radians(p_curr['lat'])))
            dy = math.sin(norm_angle) * (LANE_WIDTH / 2) / m_per_deg
            
            left_boundary_nodes.append(add_node(p_curr['lat'] - dy, p_curr['lon'] - dx))
            right_boundary_nodes.append(add_node(p_curr['lat'] + dy, p_curr['lon'] + dx))

        def create_way(node_ids, is_boundary=False):
            nonlocal way_id_counter
            way = ET.SubElement(osm_root, 'way', id=str(way_id_counter), visible='true', version='1')
            way_id_counter += 1
            for node_id in node_ids:
                ET.SubElement(way, 'nd', ref=str(node_id))
            if is_boundary:
                ET.SubElement(way, 'tag', k='type', v='line_thin')
                ET.SubElement(way, 'tag', k='subtype', v='solid')
                ET.SubElement(way, 'tag', k='width', v=LINE_WIDTH_TAG)
            return way

        left_way = create_way(left_boundary_nodes, is_boundary=True)
        right_way = create_way(right_boundary_nodes, is_boundary=True)
        center_way = create_way(centerline_way_nodes)
        
        relation = ET.SubElement(osm_root, 'relation', id=str(relation_id_counter), visible='true', version='1')
        relation_id_counter += 1
        ET.SubElement(relation, 'member', type='way', ref=left_way.get('id') or '', role='left')
        ET.SubElement(relation, 'member', type='way', ref=right_way.get('id') or '', role='right')
        ET.SubElement(relation, 'member', type='way', ref=center_way.get('id') or '', role='centerline')
        ET.SubElement(relation, 'tag', k='type', v='lanelet')
        ET.SubElement(relation, 'tag', k='subtype', v='road')

    xml_str = ET.tostring(osm_root, 'utf-8')
    reparsed = minidom.parseString(xml_str)
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(reparsed.toprettyxml(indent='  '))

    print(f"Generated Lanelet2 map at {output_file}")

if __name__ == '__main__':
    osm_files = [
        'src/core/localization/localization_core/data/mirae_link.osm',
        'src/core/localization/localization_core/data/mirae_intersection.osm'
    ]
    output_osm_file = 'mirae_lanelet2_map.osm'
    generate_lanelet2_map(osm_files, output_osm_file) 