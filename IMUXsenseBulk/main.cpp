/* -------------------------------------------------------------------------- *
 *                            OpenSim:  main.cpp                              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2024 Stanford University and the Authors * Author(s): Alex
 * Beattie, Ayman Habib, Ajay Seth                            *
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

#include <OpenSim/Common/ExperimentalSensor.h>
#include <OpenSim/Common/XsensDataReader.h>
#include <OpenSim/Common/XsensDataReaderSettings.h>

#include <chrono> // for std::chrono functions
#include <clocale>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

namespace fs = std::filesystem;

const std::vector<OpenSim::ExperimentalSensor> experimentalSensors = {
    OpenSim::ExperimentalSensor("_00B42DA3", "pelvis_imu"),
    OpenSim::ExperimentalSensor("_00B42DAE", "tibia_r_imu"),
    OpenSim::ExperimentalSensor("_00B42DA2", "femur_r_imu"),
    OpenSim::ExperimentalSensor("_00B42D53", "tibia_l_imu"),
    OpenSim::ExperimentalSensor("_00B42D4D", "femur_l_imu"),
    OpenSim::ExperimentalSensor("_00B42D48", "calcn_r_imu"),
    OpenSim::ExperimentalSensor("_00B42D4E", "calcn_l_imu")};

void process(const fs::path &file) {
  std::cout << "---Starting Processing: " << file << std::endl;
  try {
    // Xsense Reader Settings
    OpenSim::XsensDataReaderSettings settings =
        OpenSim::XsensDataReaderSettings();
    settings.set_ExperimentalSensors(experimentalSensors);
    settings.set_data_folder(file.parent_path());
    settings.set_trial_prefix(file.stem());
    OpenSim::XsensDataReader reader(settings);

    std::string folder = settings.get_data_folder() + "/";
    OpenSim::DataAdapter::OutputTables tables = reader.read(folder);

    // Orientations
    const OpenSim::TimeSeriesTableQuaternion &quatTableTyped =
        reader.getOrientationsTable(tables);
    const std::string orientationsOutputPath = folder + settings.get_trial_prefix() + "_orientations.sto";
    OpenSim::STOFileAdapter_<SimTK::Quaternion>::write(quatTableTyped, orientationsOutputPath);
    std::cout << "\tWrote'" << orientationsOutputPath << std::endl;
   
  } catch (...) {
    std::cout << "Error in processing File: " << file << std::endl;
  }
  std::cout << "---Ending Processing: " << file << std::endl;
}

void processDirectory(const fs::path &dirPath) {

  std::vector<std::thread> threads;
  // Iterate through the directory
  for (const auto &entry : fs::directory_iterator(dirPath)) {
    if (entry.is_directory()) {
      // Recursively process subdirectory
      processDirectory(entry.path());
    } else if (entry.is_regular_file()) {
      // Check if the file has a .mat extension
      if (entry.path().extension() == ".mat") {
        // Create a corresponding text file
        fs::path textFilePath = entry.path();
        threads.emplace_back(process, textFilePath);
      }
    }
  }
  // Join all threads
  for (auto &thread : threads) {
    if (thread.joinable()) {
      thread.join();
    }
  }
}

int main(int argc, char *argv[]) {
  std::chrono::steady_clock::time_point begin =
      std::chrono::steady_clock::now();
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
  std::cout << "Runtime = "
            << std::chrono::duration_cast<std::chrono::microseconds>(end -
                                                                     begin)
                   .count()
            << "[µs]" << std::endl;
  std::cout << "Finished Running without Error!" << std::endl;
  return 0;
}