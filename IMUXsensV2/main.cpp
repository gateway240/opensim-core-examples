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
#include <OpenSim/Common/XsensDataReader.h>
#include <OpenSim/OpenSim.h>
#include <iostream>
#include <string>
#include <vector>

int main() {
  const std::vector<std::pair<std::string, std::string>> settings_files = {
      {"myIMUMappings.xml", "tab"},
      {"myIMUMappings.xml", "comma"},
      {"myIMUMappings.xml", "space"},
  };
  for (const auto &p : settings_files) {
    const std::string settingsFile = p.first;
    const std::string delim = p.second;
    std::cout << "Starting with: " << delim << std::endl;

    OpenSim::XsensDataReaderSettings readerSettings(settingsFile);
    const std::string trial_prefix =
        readerSettings.get_trial_prefix() + "_" + delim;
    readerSettings.set_trial_prefix(trial_prefix);
    if (delim.compare("comma") == 0) {
      readerSettings.set_delimiter(",");
      readerSettings.set_rotation_representation("rot_euler");
    } else if (delim.compare("space") == 0) {
      readerSettings.set_delimiter(" ");
      readerSettings.set_rotation_representation("rot_quaternion");
    }
    OpenSim::XsensDataReader reader(readerSettings);
    std::string folder = readerSettings.get_data_folder();
    OpenSim::DataAdapter::OutputTables tables = reader.read(folder);

    // Magnetometer
    const OpenSim::TimeSeriesTableVec3 &magTableTyped =
        reader.getMagneticHeadingTable(tables);
    OpenSim::STOFileAdapterVec3::write(magTableTyped,
                                       trial_prefix + "_magnetometers.sto");
    // Accelerometer
    const OpenSim::TimeSeriesTableVec3 &accelTableTyped =
        reader.getLinearAccelerationsTable(tables);
    OpenSim::STOFileAdapterVec3::write(accelTableTyped,
                                       trial_prefix + "_accelerations.sto");
    // Gyro
    const OpenSim::TimeSeriesTableVec3 &gyroTableTyped =
        reader.getAngularVelocityTable(tables);
    OpenSim::STOFileAdapterVec3::write(gyroTableTyped,
                                       trial_prefix + "_gyros.sto");
    // Orientations
    const OpenSim::TimeSeriesTableQuaternion &quatTableTyped =
        reader.getOrientationsTable(tables);

    OpenSim::STOFileAdapter_<SimTK::Quaternion>::write(
        quatTableTyped, trial_prefix + "_orientations.sto");
  }

  return 0;
}
