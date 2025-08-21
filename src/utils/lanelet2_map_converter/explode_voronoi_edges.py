import geopandas as gpd
from shapely.geometry import LineString
import argparse

def main():
    parser = argparse.ArgumentParser(description="Explode polygon boundaries from a GeoJSON file into individual, unique line segments.")
    parser.add_argument("--input_file", required=True, help="Path to the GeoJSON file containing closed LineStrings (polygon boundaries).")
    parser.add_argument("--output_file", required=True, help="Path for the output GeoJSON file with individual line segments.")
    args = parser.parse_args()

    try:
        gdf = gpd.read_file(args.input_file)
    except Exception as e:
        print(f"Error reading file {args.input_file}: {e}")
        return

    individual_segments = []
    seen_segments_wkt = set()

    print(f"Processing {len(gdf)} features...")
    for index, feature in gdf.iterrows():
        if not isinstance(feature.geometry, LineString):
            continue

        coords = list(feature.geometry.coords)
        for i in range(len(coords) - 1):
            p1 = coords[i]
            p2 = coords[i+1]
            
            # Create a segment
            segment = LineString([p1, p2])

            # To handle duplicates regardless of direction (A->B vs B->A),
            # we create a canonical representation of the segment using sorted coordinates.
            sorted_coords = tuple(sorted((p1, p2)))
            
            if sorted_coords not in seen_segments_wkt:
                seen_segments_wkt.add(sorted_coords)
                individual_segments.append(segment)

    if not individual_segments:
        print("No segments were generated. The input file might be empty or contain no LineStrings.")
        return

    # Create a new GeoDataFrame and save it
    output_gdf = gpd.GeoDataFrame(geometry=individual_segments, crs=gdf.crs)
    try:
        output_gdf.to_file(args.output_file, driver='GeoJSON')
        print(f"Successfully created {len(output_gdf)} unique segments in {args.output_file}")
    except Exception as e:
        print(f"Error writing file {args.output_file}: {e}")

if __name__ == "__main__":
    main()
