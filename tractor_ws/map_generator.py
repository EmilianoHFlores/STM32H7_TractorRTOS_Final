# map_generator.py

import numpy as np

def generate_map(width, height, resolution, output_path):
    # Create an empty grid
    grid = np.ones((height, width), dtype=np.uint8) * 255  # Grey color outside the walls

    # Define the wall thickness in pixels
    wall_thickness = 10
    grey_area_thickness = 50

    # walls are between grey and white area
    # Define the wall locations
    # walls are between grey and white area
    # Define the wall locations
    # left wall
    grid[:, grey_area_thickness:grey_area_thickness+wall_thickness] = 0
    # right wall
    grid[:, -wall_thickness-grey_area_thickness: - grey_area_thickness] = 0
    # bottom wall
    grid[-wall_thickness - grey_area_thickness : - grey_area_thickness, :] = 0
    # top wall
    grid[grey_area_thickness:wall_thickness+grey_area_thickness, :] = 0

    # Define the grey area locations
    # left grey area
    grid[:, :grey_area_thickness] = 190
    # right grey area
    grid[:, -grey_area_thickness:] = 190
    # bottom grey area
    grid[-grey_area_thickness:, :] = 190
    # top grey area
    grid[:grey_area_thickness, :] = 190


    # Save the grid as a PGM file with the proper header
    pgm_filename = output_path + ".pgm"
    with open(pgm_filename, 'w') as pgm_file:
        pgm_file.write("P2\n")
        pgm_file.write("# Simple Map\n")
        pgm_file.write("{} {}\n".format(width, height))
        pgm_file.write("255\n")
        np.savetxt(pgm_file, grid, fmt='%d')

    # Create a YAML file to describe the map
    yaml_filename = output_path + ".yaml"
    with open(yaml_filename, 'w') as yaml_file:
        yaml_file.write("image: " + pgm_filename + "\n")
        yaml_file.write("resolution: " + str(resolution) + "\n")
        yaml_file.write("origin: [0.0, 0.0, 0.0]\n")
        yaml_file.write("negate: 0\n")
        yaml_file.write("occupied_thresh: 0.65\n")
        yaml_file.write("free_thresh: 0.196\n")

    print("Map generated successfully!")

if __name__ == "__main__":
    # Define map parameters
    map_width = 2800  # in millimeters
    map_height = 1800  # in millimeters
    resolution = 0.001  # in meters per pixel

    # Output map files to the current directory
    output_path = "map_with_grey_area"

    generate_map(map_width, map_height, resolution, output_path)
