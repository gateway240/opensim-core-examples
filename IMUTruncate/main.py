import os

def truncate_data_files(directory):
    for filename in os.listdir(directory):
        if filename.startswith("MT46_") or filename.startswith("MT2022_"):
            filepath = os.path.join(directory, filename)
            if os.path.isfile(filepath):
                with open(filepath, 'r') as file:
                    lines = file.readlines()

                # Separate header and data
                header_lines = []
                data_lines = []
                in_data_section = False

                for i, line in enumerate(lines):
                    if line.strip().startswith("//"):
                        header_lines.append(line)
                    elif not in_data_section:
                        header_lines.append(line)  # Column header line
                        in_data_section = True
                    else:
                        data_lines.append(line)

                # Keep only 5 data lines
                truncated_data = data_lines[:5]

                # Write back to file
                with open(filepath, 'w') as file:
                    file.writelines(header_lines + truncated_data)

                print(f"Truncated: {filename} to 5 data lines.")

# Run the function on your directory
truncate_data_files("/home/alexbeat/opensim-workspace/opensim-core-source/OpenSim/Tests/shared/IMUData/Xsens")
