/* -------------------------------------------------------------------------- *
 *                            OpenSim:  main.cpp                              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2024 Stanford University and the Authors                *
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

const std::string dirData = "data";
const std::string fileNameParticipants = "info_participants.csv";
// Rotation from marker space to OpenSim space (y is up)
const SimTK::Vec3 rotations(-SimTK::Pi/2,0,0);
const SimTK::Rotation sensorToOpenSim = SimTK::Rotation(
    SimTK::BodyOrSpaceType::SpaceRotationSequence, rotations[0], SimTK::XAxis,
      rotations[1], SimTK::YAxis, rotations[2], SimTK::ZAxis);

const std::vector<OpenSim::ExperimentalSensor> expSens1and2 = {
    OpenSim::ExperimentalSensor("_00B42DA3", "pelvis_imu"),
    OpenSim::ExperimentalSensor("_00B42DAE", "tibia_r_imu"),
    OpenSim::ExperimentalSensor("_00B42DA2", "femur_r_imu"),
    OpenSim::ExperimentalSensor("_00B42D53", "tibia_l_imu"),
    OpenSim::ExperimentalSensor("_00B42D4D", "femur_l_imu"),
    OpenSim::ExperimentalSensor("_00B42D48", "calcn_r_imu"),
    OpenSim::ExperimentalSensor(
        "_00B42D4E", "calcn_l_imu") // Note subject 1 and 2 have this IMU
};

const std::vector<OpenSim::ExperimentalSensor> expSensRemaining = {
    OpenSim::ExperimentalSensor("_00B42DA3", "pelvis_imu"),
    OpenSim::ExperimentalSensor("_00B42DAE", "tibia_r_imu"),
    OpenSim::ExperimentalSensor("_00B42DA2", "femur_r_imu"),
    OpenSim::ExperimentalSensor("_00B42D53", "tibia_l_imu"),
    OpenSim::ExperimentalSensor("_00B42D4D", "femur_l_imu"),
    OpenSim::ExperimentalSensor("_00B42D48", "calcn_r_imu"),
    OpenSim::ExperimentalSensor("_00B42D51", "calcn_l_imu")};

void process(
    const fs::path &sourceDir, const fs::path &resultDir, const std::string &fileStem,
    const std::vector<OpenSim::ExperimentalSensor> &experimentalSensors) {
  std::cout << "---Starting Processing: " << sourceDir << " stem: " << fileStem << std::endl;
  try {
    // // Xsense Reader Settings
    // OpenSim::XsensDataReaderSettings settings =
    //     OpenSim::XsensDataReaderSettings();
    // settings.set_ExperimentalSensors(experimentalSensors);
    // settings.set_data_folder(file);
    // settings.set_trial_prefix(trial_prefix);
    // OpenSim::XsensDataReader reader(settings);

    // std::string folder = settings.get_data_folder();
    // std::cout << "Reading folder: " << folder
    //           << " Reading trial prefix: " << trial_prefix << std::endl;
    // OpenSim::DataAdapter::OutputTables tables = reader.read(folder);

    // const std::string base_filename = file.string() + settings.get_trial_prefix();
    // // Orientations
    // const OpenSim::TimeSeriesTableQuaternion &quatTableTyped =
    //     reader.getOrientationsTable(tables);
    // const std::string orientationsOutputPath = base_filename + "_orientations.sto";
    // OpenSim::STOFileAdapter_<SimTK::Quaternion>::write(quatTableTyped,
    //                                                    orientationsOutputPath);

    // std::cout << "\tWrote'" << orientationsOutputPath << std::endl;

    // // Accelerometer
    // const OpenSim::TimeSeriesTableVec3 &accelTableTyped =
    //     reader.getLinearAccelerationsTable(tables);
    // const std::string accelerationsOutputPath = base_filename + "_accelerations.sto";
    // OpenSim::STOFileAdapterVec3::write(accelTableTyped,accelerationsOutputPath);
    // std::cout << "\tWrote'" << accelerationsOutputPath << std::endl;

  } catch (const std::exception &e) {
    // Catching standard exceptions
    std::cerr << "Error in processing: " << e.what() << std::endl;
  } catch (...) {
    std::cout << "Error in processing Dir: " << sourceDir << std::endl;
  }
  std::cout << "-------Finished Result: " << resultDir << std::endl;
}

void processDirectory(const fs::path &dirPath, const fs::path &resultPath) {

  std::vector<std::thread> threads;
  // Iterate through the directory
  for (const auto &entry : fs::directory_iterator(dirPath)) {
    if (entry.is_directory()) {
      // Recursively process subdirectory
      processDirectory(entry.path(), resultPath);
    } else if (entry.is_regular_file()) {
      // Check if the file has a .mat extension
      // std::cout << entry.path().filename() << std::endl;
      if (entry.path().filename() == "calib_static_markers.trc") {
        // Create a corresponding text file
        fs::path textFilePath = entry.path();
        // Get the last two parent directories
        const std::filesystem::path firstParent = textFilePath.parent_path();
        const std::filesystem::path secondParent = firstParent.parent_path();

        const std::filesystem::path sourceDir =
            dirPath / secondParent.filename() / firstParent.filename() / "";
        const std::filesystem::path resultDir =
            resultPath / secondParent.filename() / firstParent.filename() / "";

        // Per Kuopio Gait Dataset specs subjects 1 and 2 have different foot
        // IMU
        bool subject1or2 =
            secondParent.filename() == "01" || secondParent.filename() == "02";
        threads.emplace_back(process, sourceDir, resultDir, textFilePath.stem(),
                             subject1or2 ? expSens1and2 : expSensRemaining);
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
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <directory_path> <output_path>"
              << std::endl;
    return 1;
  }

  fs::path directoryPath = argv[1];
  if (!fs::exists(directoryPath) || !fs::is_directory(directoryPath)) {
    std::cerr << "The provided path is not a valid directory." << std::endl;
    return 1;
  }

  fs::path outputPath = argv[2];

  processDirectory(directoryPath, outputPath);
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  std::cout << "Runtime = "
            << std::chrono::duration_cast<std::chrono::microseconds>(end -
                                                                     begin)
                   .count()
            << "[µs]" << std::endl;
  std::cout << "Finished Running without Error!" << std::endl;
  return 0;
}