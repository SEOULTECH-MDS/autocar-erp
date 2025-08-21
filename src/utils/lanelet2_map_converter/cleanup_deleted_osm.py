import xml.etree.ElementTree as ET
import os
from xml.dom import minidom

def cleanup_deleted_elements(input_osm, output_osm, delete_tag_key='delete'):
    """
    Reads an OSM file, removes nodes and ways with a 'delete' tag,
    and cleans up references.
    """
    print(f"Loading OSM file: {input_osm}")
    try:
        tree = ET.parse(input_osm)
        root = tree.getroot()
    except ET.ParseError as e:
        print(f"Error parsing XML file: {e}")
        return

    nodes_to_delete = set()
    ways_to_delete = set()

    # --- Pass 1: Identify elements marked for deletion ---
    print("Pass 1: Identifying elements marked for deletion...")
    for way in root.findall('way'):
        if way.get('action') == 'delete':
            ways_to_delete.add(way.get('id'))

    for node in root.findall('node'):
        if node.get('action') == 'delete':
            nodes_to_delete.add(node.get('id'))
        else:
            for tag in node.findall('tag'):
                if tag.get('k') == delete_tag_key:
                    nodes_to_delete.add(node.get('id'))
                    break

    for way in root.findall('way'):
        if way.get('id') not in ways_to_delete:
            for tag in way.findall('tag'):
                if tag.get('k') == delete_tag_key:
                    ways_to_delete.add(way.get('id'))
                    break

    print(f"Found {len(ways_to_delete)} ways and {len(nodes_to_delete)} nodes tagged for deletion.")

    if not ways_to_delete and not nodes_to_delete:
        print("No elements with delete tag found. Nothing to do.")
        return

    # --- Pass 2: Update ways that reference deleted nodes and collect all valid node references ---
    print("Pass 2: Cleaning up references in ways...")
    all_referenced_nodes = set()
    
    for way in root.findall('way'):
        if way.get('id') in ways_to_delete:
            continue

        nodes_in_way = way.findall('nd')
        nds_to_remove_from_way = []
        for nd in nodes_in_way:
            ref_id = nd.get('ref')
            if ref_id in nodes_to_delete:
                nds_to_remove_from_way.append(nd)
            else:
                all_referenced_nodes.add(ref_id)
        
        for nd in nds_to_remove_from_way:
            way.remove(nd)
    
    # --- Pass 3: Identify orphaned nodes ---
    print("Pass 3: Identifying orphaned nodes...")
    for node in root.findall('node'):
        node_id = node.get('id')
        if node_id in nodes_to_delete:
            continue
        
        if node_id not in all_referenced_nodes:
            has_meaningful_tag = False
            for tag in node.findall('tag'):
                if tag.get('k') not in ['mgrs_code', delete_tag_key]:
                     has_meaningful_tag = True
                     break
            
            if not has_meaningful_tag:
                nodes_to_delete.add(node_id)
    
    print(f"Total nodes to delete after finding orphans: {len(nodes_to_delete)}")

    # --- Pass 4: Remove identified elements from the tree ---
    print("Pass 4: Removing elements from the tree...")
    
    ways_to_remove = [way for way in root.findall('way') if way.get('id') in ways_to_delete]
    for way in ways_to_remove:
        root.remove(way)

    nodes_to_remove = [node for node in root.findall('node') if node.get('id') in nodes_to_delete]
    for node in nodes_to_remove:
        root.remove(node)

    # --- Write output ---
    print(f"Writing cleaned OSM to: {output_osm}")
    xml_str = ET.tostring(root, 'utf-8')
    reparsed = minidom.parseString(xml_str)
    with open(output_osm, 'w', encoding='utf-8') as f:
        f.write(reparsed.toprettyxml(indent='  '))

    print("Cleanup complete.")


if __name__ == '__main__':
    input_file = '/home/hmmdyn/autocar-erp/autocar_lanelet2_map1.osm'
    output_file = '/home/hmmdyn/autocar-erp/autocar_lanelet2_map_cleaned.osm'
    
    if not os.path.exists(input_file):
        print(f"Error: Input file not found at {input_file}")
    else:
        cleanup_deleted_elements(input_file, output_file)
