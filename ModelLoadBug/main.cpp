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
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Tools/GenericModelMaker.h>

#include <memory>
#include <string>
#include <filesystem>
#include <iostream>
#include <clocale>
#include <chrono> // for std::chrono functions


int main()
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    const std::string fileNameModel = "gait2392_thelen2003muscle.osim";
    const std::string fileNameMarkerSetBroken = "kg_gait2392_thelen2003muscle_Scale_MarkerSet_BROKEN.xml";
    const std::string fileNameMarkerSetWorking = "kg_gait2392_thelen2003muscle_Scale_MarkerSet_WORKING.xml";
    // const std::string fileNameMarkerSet = fileNameMarkerSetBroken;

    const std::string _pathToSubject = OpenSim::IO::getParentDirectory(fileNameModel);

    // Only using 2 for demonstration purposes
    const int iterations = 2;

    std::unique_ptr<OpenSim::Model> model;
    // Initializing the model works
    for (int i = 0; i < iterations; i++) {
        try {
            // Works fine
            std::cout << "\nInitializing Model Regularly!" << std::endl;
            std::string modelPath = 
                SimTK::Pathname::getAbsolutePathnameUsingSpecifiedWorkingDirectory(_pathToSubject, fileNameModel);
            model = std::make_unique<OpenSim::Model>(OpenSim::Model(modelPath));
            // std::unique_ptr<OpenSim::Model> model = std::make_unique<OpenSim::Model>(OpenSim::Model(fileNameModel));
            model->initSystem();
            std::string markerSetPath = 
                SimTK::Pathname::getAbsolutePathnameUsingSpecifiedWorkingDirectory(_pathToSubject, fileNameMarkerSetWorking);
            std::cout << "Loading marker set from: " << markerSetPath << std::endl;
            OpenSim::MarkerSet markerSet(markerSetPath);
            model->updateMarkerSet(markerSet);
            std::cout << "[" << i << "] Iteration Completed!" << std::endl;
        }
        catch (const OpenSim::Exception& x)
        {
            std::cerr << x.what() << std::endl;
        } 
    }

    std::cout << "\n\nTesting GenericModelMaker!" << std::endl;
    for (int i = 0; i < iterations; i++) {
        std::cout << "\nInitializing Model with GenericModelMaker!" << std::endl;
        std::unique_ptr<OpenSim::GenericModelMaker> genericModelMaker = std::make_unique<OpenSim::GenericModelMaker>(OpenSim::GenericModelMaker());
        genericModelMaker->setModelFileName(fileNameModel);
        genericModelMaker->setMarkerSetFileName(fileNameMarkerSetWorking);

        std::unique_ptr<OpenSim::Model> genericModel(genericModelMaker->processModel(_pathToSubject));
    
        std::cout << "[" << i << "] Iteration Completed!" << std::endl;
    }

   
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Runtime = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    std::cout << "Finished Running without Error!" << std::endl;
    // return 0;
}
