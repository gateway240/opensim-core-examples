# Alex Beattie's .osim Model Triangle Inequality Repair Script

import os
from concurrent.futures import ThreadPoolExecutor
from collections import defaultdict

import xml.etree.ElementTree as ET

# directory_to_traverse = "/home/alexbeat/AlexDev/opensim-models"  # Change this to your target directory
directory_to_traverse = "/home/alexbeat/opensim-workspace/opensim-core-source"  # Change this to your target directory

def find_and_replace(file_path, search_key, old_substring, new_substring):
    try:
        # Read the contents of the file
        with open(file_path, 'r') as file:
            lines = file.readlines()

        # Flag to check if the line was found and modified
        modified = False

        # Iterate through the lines and perform the replacement
        for i in range(len(lines)):
            if search_key in lines[i] and old_substring in lines[i]:
                # Replace the old substring with the new substring
                lines[i] = lines[i].replace(old_substring, new_substring)
                modified = True
                print(f"Modified line: {lines[i].strip()}")  # Print the modified line

        # If a modification was made, write the changes back to the file
        if modified:
            with open(file_path, 'w') as file:
                file.writelines(lines)
            print("File updated successfully.")
        else:
            print("No lines containing the search key were found.")

    except FileNotFoundError:
        print(f"The file {file_path} does not exist.")
    except Exception as e:
        print(f"An error occurred: {e}")

def check_triangle_inequality(Ixx, Iyy, Izz, slop=2e-16):
    """Check the triangle inequality for inertia matrix."""
    return (Ixx + Iyy + slop >= Izz) and (Ixx + Izz + slop >= Iyy) and (Iyy + Izz + slop >= Ixx)

def compute_minimum_change_single_element(Ixx, Iyy, Izz):
    # Initialize variables to track the minimum change and the corresponding new values
    min_change = float('inf')
    modified_element = None
    modified_element_original = -1
    modified_element_value = -1
    new_values = (Ixx, Iyy, Izz)

    # Check if modifying Ixx can satisfy the inequalities
    if Iyy + Izz > Ixx:
        change = Iyy + Izz - Ixx
        if change < min_change:
            min_change = change
            modified_element = 'Ixx'
            modified_element_original = Ixx
            modified_element_value = Iyy + Izz
            new_values = (modified_element_value, Iyy, Izz)  # New Ixx value

    # Check if modifying Iyy can satisfy the inequalities
    if Ixx + Izz > Iyy:
        change = Ixx + Izz - Iyy
        if change < min_change:
            min_change = change
            modified_element = 'Iyy'
            modified_element_original = Iyy
            modified_element_value = Ixx + Izz
            new_values = (Ixx, modified_element_value, Izz)  # New Iyy value

    # Check if modifying Izz can satisfy the inequalities
    if Ixx + Iyy > Izz:
        change = Ixx + Iyy - Izz
        if change < min_change:
            min_change = change
            modified_element = 'Izz'
            modified_element_original = Izz
            modified_element_value = Ixx + Iyy
            new_values = (Ixx, Iyy, modified_element_value)  # New Izz value

    # Return the minimum change needed, the modified element, and the new values
    return min_change if min_change != float('inf') else 0, modified_element, modified_element_original, modified_element_value, new_values

def parse_osim_file(osim_file):
    print(f"\n----- Processing file: {osim_file} ------")
    try:
        tree = ET.parse(osim_file)
        root = tree.getroot()

        # Find all inertia elements
        inertia_elements = root.findall('.//inertia')

        if not inertia_elements:
            print("No inertia elements found in the .osim file.")
            return

        # not_satisfied = []  # List to store values where triangle inequality is NOT satisfied

        for inertia in inertia_elements:
            # Get the text content and split it into individual values
            values = list(map(float, inertia.text.split()))
            if len(values) < 6:
                print("Not enough values found in inertia element.")
                continue
            
            Ixx, Iyy, Izz = values[0], values[1], values[2]

            # print(f"Found inertia: Ixx={Ixx}, Iyy={Iyy}, Izz={Izz}")

            if not check_triangle_inequality(Ixx, Iyy, Izz):
                print("\nTriangle inequality NOT satisfied for the following values:")
                print(f"Ixx={Ixx}, Iyy={Iyy}, Izz={Izz}")
                min_change, modified_element, modified_element_original, modified_element_value, new_values = compute_minimum_change_single_element(Ixx, Iyy, Izz)
                print(f"Ixx={new_values[0]}, Iyy={new_values[1]}, Izz={new_values[2]} ==> minimum change (modifying {modified_element}) by: {min_change} ")
                
                print(f"Inequality now satisfied: {check_triangle_inequality(new_values[0], new_values[1], new_values[2])}\n")
                find_and_replace(osim_file,'<inertia>', str(modified_element_original), str(modified_element_value))


    except ET.ParseError as e:
        print(f"Error parsing the file: {e}")
    except Exception as e:
        print(f"An error occurred: {e}")

def traverse_directory(directory):
    """Recursively traverse the directory and return a list of file paths."""
    file_paths = []
    for root, _, files in os.walk(directory):
        for file in files:
            file_paths.append(os.path.join(root, file))
    return file_paths


def main(directory):
    """Main function to traverse the directory and process files using a task pool."""
    # Get all file paths in the directory
    file_paths = traverse_directory(directory)
    files = [file for file in file_paths if file.endswith(".osim")]

    # Determine the number of available threads
    max_workers = os.cpu_count()  # Get the number of CPU cores
    # max_workers = 1
    print(f"Using {max_workers} threads.")

    # Use a thread pool to process files concurrently
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        # Map the process_file function to the file paths
        results = list(executor.map(parse_osim_file, files))


if __name__ == "__main__":
    print("---Beginning Script!---")
    # Specify the directory to traverse
    main(directory_to_traverse)
    print("---Ending Script!---")