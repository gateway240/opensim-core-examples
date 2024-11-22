/* -------------------------------------------------------------------------- *
 *                            OpenSim:  main.cpp                              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2024 Stanford University and the Authors                     *
 * Author(s): Alex Beattie, Ayman Habib, Ajay Seth                            *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */


// INCLUDES
#include <OpenSim/Common/C3DFileAdapter.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/TRCFileAdapter.h>

#include <iostream>
#include <filesystem>
#include <fstream>
#include <string>
#include <iostream>
#include <clocale>
#include <chrono> // for std::chrono functions

namespace fs = std::filesystem;

void processC3DFile(std::string filename) {
    std::cout << "---Starting Processing: " << filename << std::endl;
    OpenSim::C3DFileAdapter c3dFileAdapter{};
    auto tables = c3dFileAdapter.read(filename);

    std::shared_ptr<OpenSim::TimeSeriesTableVec3> marker_table = c3dFileAdapter.getMarkersTable(tables);
    std::shared_ptr<OpenSim::TimeSeriesTableVec3> force_table = c3dFileAdapter.getForcesTable(tables);
    std::shared_ptr<OpenSim::TimeSeriesTable> analog_table = c3dFileAdapter.getAnalogDataTable(tables);

    size_t ext = filename.rfind(".");
    std::string base = filename.substr(0, ext);

    const std::string marker_file = base + "_markers.trc";
    const std::string forces_file = base + "_grfs.sto";
    const std::string analogs_file = base + "_analog.sto";

    // Write marker locations
    marker_table->updTableMetaData().setValueForKey("Units", 
                                                    std::string{"mm"});
    OpenSim::TRCFileAdapter trc_adapter{};
    trc_adapter.write(*marker_table, marker_file);
    std::cout << "\tWrote '" << marker_file << std::endl;

    // Write forces and analog 
    OpenSim::STOFileAdapter sto_adapter{};
    sto_adapter.write((force_table->flatten()), forces_file);
    std::cout << "\tWrote'" << forces_file << std::endl;
    sto_adapter.write(*analog_table, analogs_file);
    std::cout << "\tWrote'" << analogs_file << std::endl;
    std::cout << "---Ending Processing: " << filename << std::endl;
}

void processDirectory(const fs::path& dirPath) {
    // Iterate through the directory
    for (const auto& entry : fs::directory_iterator(dirPath)) {
        if (entry.is_directory()) {
            // Recursively process subdirectory
            processDirectory(entry.path());
        } else if (entry.is_regular_file()) {
            // Check if the file has a .c3d extension
            if (entry.path().extension() == ".c3d") {
                // Create a corresponding text file
                fs::path textFilePath = entry.path();
                // Convert fs::path to std::string
                std::string pathString = textFilePath .string();
                try {
                    processC3DFile(pathString);
                } catch (...) {
                    std::cout << "Error in processing C3D File: " << pathString << std::endl;
                }
            }
        }
    }
}

int main(int argc, char* argv[]) {
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <directory_path>" << std::endl;
        return 1;
    }

    fs::path directoryPath = argv[1];

    if (!fs::exists(directoryPath) || !fs::is_directory(directoryPath)) {
        std::cerr << "The provided path is not a valid directory." << std::endl;
        return 1;
    }

    processDirectory(directoryPath);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Runtime = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    std::cout << "Finished Running without Error!" << std::endl;
    return 0;
}