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
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/TRCFileAdapter.h>
#include <OpenSim/Tools/ScaleTool.h>


#include <string>
#include <sstream>
#include <filesystem>
#include <iostream>
#include <clocale>
#include <chrono> // for std::chrono functions
#include <vector>

struct Participant {
    int ID; // study id
    int Age; // 1-...
    int Height; // in cm
    double Mass; // in kg
};

const std::string dirData = "data";
const std::string fileNameParticipants = "info_participants.csv";
const std::string fileNameCalibration = "subject01_calib_static_markers.trc";
// Rotation from marker space to OpenSim space (y is up)
// This is the rotation for the kuopio gait dataset
const SimTK::Vec3 rotations(-SimTK::Pi/2,SimTK::Pi/2,0);

// Function to rotate a table of Vec3 elements
void rotateMarkerTable(
        OpenSim::TimeSeriesTableVec3& table,
        const SimTK::Rotation_<double>& rotationMatrix)
{
    const SimTK::Rotation R_XG = rotationMatrix;

    int nc = int(table.getNumColumns());
    size_t nt = table.getNumRows();

    for (size_t i = 0; i < nt; ++i) {
        auto row = table.updRowAtIndex(i);
        for (int j = 0; j < nc; ++j) {
            row[j] = R_XG * row[j];
        }
    }
    return;
}

int main()
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    // SET OUTPUT FORMATTING
    OpenSim::IO::SetDigitsPad(4);

    // Got Participant information from spreadsheet
    Participant subject01 = Participant{
        1,38,175, 75.5278615475429
    };

    // ROTATE the marker table so the orientation is correct
    // Create a new filename by modifying the original path
    std::filesystem::path newFilePath = fileNameCalibration; // Start with the original path

    // Get the filename without extension
    std::string newFileName = newFilePath.stem().string() + "_rotated" + newFilePath.extension().string();

    OpenSim::TRCFileAdapter trcfileadapter{};
    OpenSim::TimeSeriesTableVec3 table{ fileNameCalibration };
    
    const SimTK::Rotation sensorToOpenSim = SimTK::Rotation(
    SimTK::BodyOrSpaceType::SpaceRotationSequence, rotations[0], SimTK::XAxis,
      rotations[1], SimTK::YAxis, rotations[2], SimTK::ZAxis);
    rotateMarkerTable(table, sensorToOpenSim);
    trcfileadapter.write(table, newFileName);

    // Construct model and read parameters file
    const std::string fileNameSetupScale = "subject01_kg_gait_gait2392_thelen2003muscle_Setup_Scale.xml";
    // const std::filesystem::path dirPath = dirData;
    // const std::filesystem::path p = dirPath / fileNameSetupScale;
    std::unique_ptr<OpenSim::ScaleTool> subject = std::make_unique<OpenSim::ScaleTool>(OpenSim::ScaleTool(fileNameSetupScale));
    subject->setSubjectMass(subject01.Mass);
    subject->setSubjectHeight(subject01.Height);
    subject->setSubjectAge(subject01.Age);

    // Keep track of the folder containing setup file, will be used to locate results to compare against
    const std::string setupFilePath=subject->getPathToSubject();

    subject->run();

   
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Runtime = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    std::cout << "Finished Running without Error!" << std::endl;
    // return 0;
}
