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
#include "SimTKcommon.h"
#include <OpenSim/Common/IO.h>


#include <clocale>
#include <chrono> // for std::chrono functions
#include <filesystem>
#include <iostream>
#include <string>

const std::string fileNameSetupScale = "bin/kg_Setup_Scale.xml";
const std::string modelFileName = "bin/Rajagopal2016.osim";

int main()
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    std::string aPathToSubject = "";
    std::string testPath = "./data/test.txt";
    const auto participantPath = std::filesystem::absolute(fileNameSetupScale);
    const auto absoluteModelPath = std::filesystem::absolute(fileNameSetupScale);
    std::cout << "Absolute Path: " << participantPath << std::endl;

    std::string baseModelPath = OpenSim::IO::getParentDirectory(absoluteModelPath);
    std::string modelPath = 
            SimTK::Pathname::getAbsolutePathnameUsingSpecifiedWorkingDirectory(baseModelPath, participantPath);
    
    std::cout << "BaseModelPath: " << baseModelPath << std::endl;
    std::cout << "Participant Path: " << participantPath << std::endl;
    std::cout << "SimTK Absolute Path: " << modelPath << std::endl;

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Runtime = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    std::cout << "Finished Running without Error!" << std::endl;
    return 0;
}
