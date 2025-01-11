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

#include <algorithm> // For std::find_if
#include <chrono> // for std::chrono functions
#include <clocale>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;


const std::string dirData = "data";
const std::string fileNameParticipants = "info_participants.csv";
const std::string fileNameCalibration = "calib_static_markers.trc";
const std::string fileNameModel = "gait2392_thelen2003muscle.osim";
const std::string fileNameMarkerSet = "kg_gait2392_thelen2003muscle_Scale_MarkerSet.xml";
// Rotation from marker space to OpenSim space (y is up)
// This is the rotation for the kuopio gait dataset
const SimTK::Vec3 rotations(-SimTK::Pi/2,SimTK::Pi/2,0);

struct Participant {
    int ID;
    int Age;
    char Gender; // 'M' or 'F'
    char Leg; // Assuming 'L' or 'R'
    double Height;
    std::vector<std::string> Invalid_trials; // Store invalid trials as a vector of strings
    int IAD;
    int Left_knee_width;
    int Right_knee_width;
    int Left_ankle_width;
    int Right_ankle_width;
    int Left_thigh_length;
    int Right_thigh_length;
    int Left_shank_length;
    int Right_shank_length;
    double Mass;
    double ICD;
    double Left_knee_width_mocap;
    double Right_knee_width_mocap;
};

std::vector<Participant> parseCSV(const std::string& filename) {
    std::vector<Participant> participants;
    std::ifstream file(filename);
    std::string line;

    // Skip the header line
    if (std::getline(file, line)) {
        // Process each line in the CSV file
        while (std::getline(file, line)) {
            std::istringstream ss(line);
            Participant participant;
            std::string invalid_trials;

            // Read each field separated by commas
            std::string token;
            try {
              std::getline(ss, token, ','); participant.ID = std::stoi(token);
              std::getline(ss, token, ','); participant.Age = std::stoi(token);
              std::getline(ss, token, ','); participant.Gender = token[0];
              std::getline(ss, token, ','); participant.Leg = token[0]; // Assuming single character
              std::getline(ss, token, ','); participant.Height = std::stod(token);
              std::getline(ss, token, ','); participant.IAD = std::stoi(token);
              std::getline(ss, token, ','); participant.Left_knee_width = std::stoi(token);
              std::getline(ss, token, ','); participant.Right_knee_width = std::stoi(token);
              std::getline(ss, token, ','); participant.Left_ankle_width = std::stoi(token);
              std::getline(ss, token, ','); participant.Right_ankle_width = std::stoi(token);
              std::getline(ss, token, ','); participant.Left_thigh_length = std::stoi(token);
              std::getline(ss, token, ','); participant.Right_thigh_length = std::stoi(token);
              std::getline(ss, token, ','); participant.Left_shank_length = std::stoi(token);
              std::getline(ss, token, ','); participant.Right_shank_length = std::stoi(token);
              std::getline(ss, token, ','); participant.Mass = std::stod(token);
              std::getline(ss, token, ','); participant.ICD = std::stod(token);
              std::getline(ss, token, ','); participant.Left_knee_width_mocap = std::stod(token);
              std::getline(ss, token, ','); participant.Right_knee_width_mocap = std::stod(token);

              std::getline(ss, invalid_trials, ','); // Read the invalid trials as a single string
              // std::istringstream invalid_ss(invalid_trials);
              // while (std::getline(invalid_ss, token, ',')) {
              //     participant.Invalid_trials.push_back(token);
              // }
              } catch (const std::invalid_argument& e) {
                std::cerr << "Error parsing line: " << line << "\n" << e.what() << std::endl;
                continue; // Skip this line and continue
              }
            // Add the participant to the vector
            participants.push_back(participant);
        }
    }

    return participants;
}

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

std::string getTwoDigitString(int number) {
    // Check if the number is within the valid range (0 to 99)
    if (number < 0 || number > 99) {
        return "Error: Number out of range"; // Handle out of range
    }

    // Create a string stream to format the number
    std::ostringstream oss;
    oss << std::setw(2) << std::setfill('0') << number; // Format to two digits

    return oss.str(); // Return the formatted string
}

void process(
    const fs::path &sourceDir, const fs::path &resultDir, const std::string &fileStem, const Participant participant) {
  std::cout << "---Starting Processing: " << sourceDir << " stem: " << fileStem << " Participant: " << participant.ID << std::endl;
  try {
    // OpenSim::IO::SetDigitsPad(4);

    // Create a new filename by modifying the original path
    const std::filesystem::path calibFilePath = sourceDir / fileNameCalibration; // Start with the original path

    // ROTATE the marker table so the orientation is correct
    OpenSim::TRCFileAdapter trcfileadapter{};
    OpenSim::TimeSeriesTableVec3 table{ calibFilePath.string() };
    
    const SimTK::Rotation sensorToOpenSim = SimTK::Rotation(
    SimTK::BodyOrSpaceType::SpaceRotationSequence, rotations[0], SimTK::XAxis,
      rotations[1], SimTK::YAxis, rotations[2], SimTK::ZAxis);
    rotateMarkerTable(table, sensorToOpenSim);

    // Get the filename without extension
    const std::string rotatedCalibFilename = calibFilePath.stem().string() + "_rotated" + calibFilePath.extension().string();
    const std::filesystem::path markerFilePath = resultDir / rotatedCalibFilename;
    // std::cout << markerFilePath.string() << std::endl;
    // Write the rotated file
    // Get the parent directory

    std::filesystem::path newDirectory = markerFilePath.parent_path();
    if (!std::filesystem::exists(newDirectory)) {
        // Create the directory
        if (std::filesystem::create_directories(newDirectory)) {
            std::cout << "Directories created: " << newDirectory << std::endl;
        } else {
            std::cerr << "Failed to create directory: " << newDirectory << std::endl;
        }
    }
    const std::string markerFileName = markerFilePath.string();
    
    trcfileadapter.write(table, markerFileName);

    // Copy over the model
    const std::filesystem::path modelSourcePath(fileNameModel);
    const std::filesystem::path markerSetSourcePath(fileNameMarkerSet);
    const std::filesystem::path modelDestinationPath = newDirectory / fileNameModel;
    const std::filesystem::path markerSetDestinationPath = newDirectory / fileNameMarkerSet;
    try {
        // Copy the file to the destination directory
        std::filesystem::copy_file(modelSourcePath, modelDestinationPath, std::filesystem::copy_options::overwrite_existing);
        std::filesystem::copy_file(markerSetSourcePath, markerSetDestinationPath, std::filesystem::copy_options::overwrite_existing);
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Error copying file: " << e.what() << std::endl;
    }
    // Construct model and read parameters file
    const std::string fileNameSetupScale = "kg_gait_gait2392_thelen2003muscle_Setup_Scale.xml";
    // const std::string fileNameModelScaler = "kg_gait_gait2392_thelen2003muscle_Setup_Model_Scaler.xml";

    // std::unique_ptr<OpenSim::ModelScaler> modelScaler(new OpenSim::ModelScaler());
    // const std::filesystem::path dirPath = dirData;
    // const std::filesystem::path p = dirPath / fileNameSetupScale;
    {
      std::unique_ptr<OpenSim::ScaleTool> subject(new OpenSim::ScaleTool(fileNameSetupScale));
    
      subject->setPathToSubject((newDirectory / "").string());
      subject->setSubjectMass(participant.Mass);
      subject->setSubjectHeight(participant.Height);
      subject->setSubjectAge(participant.Age);

      subject->run();
      // Clear out the pointer
      // subject->_genericModelMaker = NULL;
      subject.reset();
    }
    // Iterate through the directory
    for (const auto& entry : std::filesystem::directory_iterator(newDirectory)) {
        if (entry.is_regular_file()) { // Check if it's a regular file
            std::string filename = entry.path().filename().string();
            std::string newFilename = filename;

            // Check if "XX" is in the filename
            size_t pos = newFilename.find("XX");
            if (pos != std::string::npos) {
                // Replace "XX" with "04"
                newFilename.replace(pos, 2, getTwoDigitString(participant.ID));

                // Create the new file path
                std::filesystem::path newFilePath = newDirectory / newFilename;

                // Rename the file
                try {
                    std::filesystem::rename(entry.path(), newFilePath);
                    std::cout << "Renamed: " << filename << " to " << newFilename << std::endl;
                } catch (const std::filesystem::filesystem_error& e) {
                    std::cerr << "Error renaming file: " << e.what() << std::endl;
                }
            }
        }
    }

  } catch (const std::exception &e) {
    // Catching standard exceptions
    std::cerr << "Error in processing: " << e.what() << std::endl;
  } catch (...) {
    std::cout << "Error in processing Dir: " << sourceDir << std::endl;
  }

  std::cout << "-------Finished Result: " << resultDir << std::endl;
}

void processDirectory(const fs::path &dirPath, const fs::path &resultPath, std::vector<Participant>& participants) {

  std::vector<std::thread> threads;
  // Iterate through the directory
  for (const auto &entry : fs::directory_iterator(dirPath)) {
    if (entry.is_directory()) {
      // Recursively process subdirectory
      processDirectory(entry.path(), resultPath, participants);
    } else if (entry.is_regular_file()) {
      // Check if the file has a .mat extension
      // std::cout << entry.path().filename() << std::endl;
      if (entry.path().filename() == "calib_static_markers.trc") {
        // Create a corresponding text file
        fs::path textFilePath = entry.path();
        // Get the last two parent directories
        const std::filesystem::path firstParent = textFilePath.parent_path();
        const std::filesystem::path secondParent = firstParent.parent_path();

        const std::filesystem::path resultDir =
            resultPath / secondParent.filename() / firstParent.filename() / "";
        const int participantId = std::stoi(secondParent.filename());
        // std::cout << "Participant: " << participantId << std::endl;
        auto it = std::find_if(participants.begin(), 
             participants.end(), 
             [&cp = participantId]
             (const Participant& p) { return cp == p.ID; }); 
        // Find the correct participant from the participants vector
        // Output the found participant or a message if not found
        if (it != participants.end()) {
          // std::cout << "Found Participant: " << std::endl;
          const Participant participant = *it;
          // process(dirPath, resultDir, textFilePath.stem(), participant);
          threads.emplace_back(process, dirPath, resultDir, textFilePath.stem(), participant);
        }

                  
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

  std::vector<Participant> participants = parseCSV(fileNameParticipants);

  // Output the parsed data
  std::cout << "Participant List: " << std::endl;
  for (const auto& participant : participants) {
      std::cout << "ID: " << participant.ID << ", Age: " << participant.Age
                << ", Gender: " << participant.Gender << ", Leg: " << participant.Leg
                << ", Height: " << participant.Height
                << ", Mass: " << participant.Mass << std::endl;
      // std::cout << "Invalid Trials: ";
      // for (const auto& trial : participant.Invalid_trials) {
      //     std::cout << trial << " ";
      // }
      // std::cout << std::endl;
  }

  processDirectory(directoryPath, outputPath, participants);
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  std::cout << "Runtime = "
            << std::chrono::duration_cast<std::chrono::microseconds>(end -
                                                                     begin)
                   .count()
            << "[µs]" << std::endl;
  std::cout << "Finished Running without Error!" << std::endl;
  return 0;
}