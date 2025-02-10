# Alex Beattie's .osim Model Triangle Inequality Repair Script

import os
import re
from concurrent.futures import ThreadPoolExecutor
from collections import defaultdict

import xml.etree.ElementTree as ET

# directory_to_traverse = "/home/alexbeat/AlexDev/opensim-models"  # Change this to your target directory
directory_to_traverse = "/home/alexbeat/opensim-workspace/opensim-core-source"  # Change this to your target directory
# directory_to_traverse = "/home/alexbeat/AlexDev/opensim-core-extension"  # Change this to your target directory


def find_and_replace(file_path, start_key, end_key, pattern, new_substring):
    try:
        # Read the contents of the file
        with open(file_path, 'r') as file:
            lines = file.readlines()

        # Flag to check if the line was found and modified
        modified = False

        # Iterate through the lines and perform the replacement
        in_block = False
        for i in range(len(lines)):
            if start_key in lines[i]:
                in_block = True
            if end_key in lines[i]:
                in_block = False
            matches = re.findall(pattern, lines[i])
            if matches and in_block:
                print("Found matches:", matches)
                lines[i] = re.sub(pattern, new_substring, lines[i])
                modified = True   
                # else:
                #     print("No matches found.")
    
              
            # if search_key in lines[i]:
              
              
                # Use re.sub to replace the matched values

                
                # Replace the old substring with the new substring

            # print(f"Modified line: {lines[i].strip()}")  # Print the modified line

        # If a modification was made, write the changes back to the file
        if modified:
            with open(file_path, 'w') as file:
                file.writelines(lines)
            print("File updated successfully.")
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

        # Find the <defaults> element
        defaults_element = root.find('.//defaults')

        elements = None
        if defaults_element is not None:
            # Find all Thelen2003Muscle elements within <defaults>
            elements = defaults_element.findall('.//Thelen2003Muscle')
        else:
            print("No Thelen2003Muscle elements found in the <defaults> section.")
            return
                
        for element in elements:
            # print(ET.tostring(element, encoding='unicode'))
            # Get the text content and split it into individual values 
            # The pattern \s*0(\.0+)?\s* matches 0 with optional trailing zeros and surrounding whitespace. 
            pattern = r'<min_control>\s*0(\.0+)?\s*</min_control>'
            start_key = '<Thelen2003Muscle name="default"'
            end_key = '</Thelen2003Muscle>'
            change = '<min_control> 0.01000000 </min_control>'
            find_and_replace(osim_file, start_key, end_key, pattern, change)


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