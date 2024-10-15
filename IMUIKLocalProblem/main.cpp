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
#include <OpenSim/Simulation/OpenSense/OpenSenseUtilities.h>
#include <OpenSim/Simulation/OpenSense/IMUPlacer.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Tools/IMUInverseKinematicsTool.h>

#include <clocale>

using namespace OpenSim;
using namespace std;

int main()
{
    // Make a "deep copy" of current locale name.
    std::string prev_loc = std::setlocale(LC_ALL, nullptr);

    // Germany = de_DE.UTF-8
    std::string locale = "fi_FI.UTF-8";
 
    // The C locale will be UTF-8 enabled Finnish with decimal dot being a comma
    if (const char* loc = std::setlocale(LC_ALL, locale.c_str())) 
        std::wprintf(L"New LC_ALL locale: %s\n", loc);
 
    std::wprintf(L"Number Format: %.2f\n", 3.14);
 

    // Calibrate model and compare result to standard
    IMUPlacer imuPlacer("imuPlacer.xml");
    imuPlacer.run();
    Model model = imuPlacer.getCalibratedModel();

    // Calibrate model from two different standing trials facing
    // opposite directions to verify that heading correction is working
    IMUPlacer placerX("imuPlacerFaceX.xml");
    placerX.run(false);
    Model facingX = placerX.getCalibratedModel();
    facingX.setName("calibrated_FacingX");
    facingX.finalizeFromProperties();

    IMUInverseKinematicsTool ik_hjc("setup_IMUInverseKinematics_HJC_trial.xml");
    ik_hjc.setModel(facingX);
    ik_hjc.set_results_directory("ik_hjc_" + facingX.getName());
    ik_hjc.run(false);

    IMUInverseKinematicsTool ik_hjc_nf("setup_IMUInverseKinematics_HJC_trial_nofeet.xml");
    ik_hjc_nf.setModel(facingX);
    ik_hjc_nf.set_results_directory("ik_hjc_nf_" + facingX.getName());
    ik_hjc_nf.run(false);

    // Now facing the opposite direction (negative X)
    IMUPlacer placerNegX("imuPlacerFaceNegX.xml");
    placerNegX.run(false);
    Model facingNegX = placerNegX.getCalibratedModel();
    facingNegX.setName("calibrated_FacingNegX");
    facingNegX.finalizeFromProperties();

    ik_hjc.setModel(facingNegX);
    ik_hjc.set_results_directory("ik_hjc_" + facingNegX.getName());
    ik_hjc.run(false);

    Storage ik_X("ik_hjc_" + facingX.getName() +
        "/ik_MT_012005D6_009-quaternions_RHJCSwinger.mot");

    Storage ik_negX("ik_hjc_" + facingNegX.getName() +
        "/ik_MT_012005D6_009-quaternions_RHJCSwinger.mot");

    int nc = ik_X.getColumnLabels().size();

    return 0;
}
