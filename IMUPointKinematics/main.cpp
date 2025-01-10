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
#include <OpenSim/Simulation/Model/Model.h>

#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Tools/AnalyzeTool.h>
#include <OpenSim/Analyses/IMUDataReporter.h>

#include <OpenSim/Analyses/PointKinematics.h>

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

    std::string coordinates_file_name = "ik_l_comf_01-000_orientations.mot";

    double start_time = 0;
    double end_time = std::numeric_limits<double>::infinity();

    // Setup Analysis Tool
    OpenSim::AnalyzeTool analyzeIMU;
    analyzeIMU.setName("test_distance_analysis");
    analyzeIMU.setModelFilename(model_name);
    analyzeIMU.setCoordinatesFileName(coordinates_file_name);
    analyzeIMU.setLowpassCutoffFrequency(-1);
    analyzeIMU.setInitialTime(start_time);
    analyzeIMU.setFinalTime(end_time);
    analyzeIMU.setResultsDir("results");

    // Setup IMU Data Reporter => Sanity Check
    OpenSim::IMUDataReporter imuDataReporter;
    imuDataReporter.setName("IMUDataReporter_no_forces");
    imuDataReporter.set_compute_accelerations_without_forces(true);
    imuDataReporter.setInDegrees(true);
    analyzeIMU.updAnalysisSet().cloneAndAppend(imuDataReporter);
    
    // Setup Point Kinematics Reporting

    // List all Bodies & IMUs
    std::cout << "\nList all IMUs in the model." << std::endl;
    int i = 0;
    const auto imus = model.getComponentList<OpenSim::IMU>();
    for (auto& component : imus) {
        std::cout << "frame[" << ++i << "] is " << component.getName()
            << " of type " << typeid(component).name() << std::endl;
    }
    
    std::cout << "\nList all bodies in the model." << std::endl;
    i = 0;
    int j = 0;
    const auto bodies = model.getComponentList<OpenSim::Body>();
    for (auto& root : bodies) {
        const std::string& root_name = root.getName();
        std::cout << "frame[" << ++i << "] is " << root_name
            << " of type " << typeid(root).name() << std::endl;
        for (auto& sub_component: bodies) {
            const std::string& sub_component_name = sub_component.getName();
            std::cout << "frame[" << ++j << "] is " << sub_component_name
                << " of type " << typeid(sub_component).name() << std::endl;
            // Create point kinematics reporter
            OpenSim::PointKinematics pointKin;
            pointKin.setPointName(root_name + "-" + sub_component_name);
            pointKin.setBody(&root);
            pointKin.setRelativeToBody(&sub_component);
            analyzeIMU.updAnalysisSet().cloneAndAppend(pointKin);
        }
    }

    // Running Analysis!
    std::string output_file_name = "test_distance_analysis_imu.xml";
    analyzeIMU.print(output_file_name);
    OpenSim::AnalyzeTool roundTrip(output_file_name);
    roundTrip.run();


    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Runtime = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    std::cout << "Finished Running without Error!" << std::endl;
    return 0;
}
