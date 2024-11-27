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
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Simulation/TableProcessor.h>
#include <OpenSim/Simulation/Model/Model.h>

#include <string>
#include <iostream>
#include <clocale>
#include <chrono> // for std::chrono functions

int main()
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    std::string model_name = "calibrated_gait2392_thelen2003muscle.osim";
    OpenSim::Model model = OpenSim::Model(model_name);
    model.initSystem();

    std::string coordinates_file_name = "ik_l_comf_01-000_orientations-unif.mot";
     // Create the coordinates table (in radians).
    auto tableProcessor =
            OpenSim::TableProcessor(coordinates_file_name) |
            OpenSim::TabOpLowPassFilter(6) |
            OpenSim::TabOpUseAbsoluteStateNames();
    auto coordinatesRadians = tableProcessor.processAndConvertToRadians(model);

    std::string output_coordinates_name = "ik_l_comf_01-000_coordinates_radians-unif.sto";
    OpenSim::STOFileAdapter::write(coordinatesRadians, output_coordinates_name);
   
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Runtime = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    std::cout << "Finished Running without Error!" << std::endl;
    return 0;
}
