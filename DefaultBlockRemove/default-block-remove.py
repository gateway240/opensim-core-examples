# Alex Beattie's .osim Default Block Removal script

import os
import re
from concurrent.futures import ThreadPoolExecutor
from collections import defaultdict

import xml.etree.ElementTree as ET

directory_to_traverse = "/home/alexbeat/AlexDev/opensim-models"  # Change this to your target directory
# directory_to_traverse = "/home/alexbeat/opensim-workspace/opensim-core-source"  # Change this to your target directory
# directory_to_traverse = "/home/alexbeat/AlexDev/opensim-core-extension"  # Change this to your target directory


def find_and_delete(file_path, start_key, end_key):
    try:
        # Read the contents of the file
        with open(file_path, 'r') as file:
            lines = file.readlines()

        # Initialize a flag to track whether we are inside the <default> tags
        in_block = False
        new_lines = []

        for line in lines:
            if start_key in line:
                in_block = True  # We found the opening tag
            if end_key in line:
                in_block = False  # We found the closing tag
                continue

            if not in_block :
                new_lines.append(line)  # Only add lines that are outside the <default> tags

        # Write the modified content back to the file
        with open(file_path, 'w') as file:
            file.writelines(new_lines)
        # else:
        #     print("No lines containing the search key were found.")

    except FileNotFoundError:
        print(f"The file {file_path} does not exist.")
    except Exception as e:
        print(f"An error occurred: {e}")


def parse_osim_file(osim_file):
    print(f"\n----- Processing file: {osim_file} ------")
    try:
        tree = ET.parse(osim_file)
        root = tree.getroot()

        # Find all inertia elements
        elements = root.findall('.//Thelen2003Muscle')

        if not elements:
            print("No elements found in the .osim file.")
            return
        # else:
                
        for element in elements:
            # print(ET.tostring(element, encoding='unicode'))
            start_key = '<defaults>'
            end_key = '</defaults>'

            find_and_delete(osim_file, start_key, end_key)


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
    files = [file for file in file_paths if (file.endswith(".osim") or file.endswith(".xml"))]

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