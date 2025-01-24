// INCLUDES
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/OpenSense/IMUPlacer.h>
#include <OpenSim/Tools/IMUInverseKinematicsTool.h>

// Thread Pool
#include "BS_thread_pool.hpp" // BS::synced_stream, BS::thread_pool

#include <algorithm> // For std::find_if
#include <chrono>    // for std::chrono functions
#include <clocale>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator> // For std::back_inserter
#include <memory>
#include <string>
#include <thread>
#include <vector>

// Logging output
const int64_t time_now =
    std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch())
        .count();
std::ofstream log_file("task-" + std::to_string(time_now) + ".log");
BS::synced_stream sync_out(std::cout, log_file);

// Configuration
typedef std::pair<std::string, std::string> ConfigType;

const std::vector<std::string> baseModels = {
    "kg_gait2392_thelen2003muscle_scaled_only.osim",
    "kg_Rajagopal2016_scaled_only.osim"};

// Simple test case
// const std::vector<std::string> includedParticipants = {"40"};

// All trials with no invalid trials
const std::vector<std::string> includedParticipants = {
    "08", "09", "12", "17", "19", "21", "24",
    "28", "31", "34", "36", "38", "40", "41"};

const std::string outputBasePrefix = "kg";
const std::string imuSuffix = "and_IMUs";

const std::string sep = "_";

void process(const std::filesystem::path &file,
             const std::filesystem::path &resultDir, const ConfigType &c) {
  sync_out.println("---Starting Model Processing: ", file.string());
  try {
    const std::filesystem::path modelSourcePath = c.second;
    const std::string modelSourceStem = modelSourcePath.stem().string();

    sync_out.println("Model Path: ", modelSourcePath.string());

    const std::string outputFilePrefix =
        outputBasePrefix + sep + file.stem().string() + sep + modelSourceStem;

    if (std::filesystem::exists(modelSourcePath)) {

      // Fix bug for set_model_file
      OpenSim::IMUPlacer imuPlacer;
      imuPlacer.set_base_imu_label("pelvis_imu");
      imuPlacer.set_base_heading_axis("-z");
      // 90 0 90
      // imuPlacer.set_sensor_to_opensim_rotations(
      //     SimTK::Vec3(-SimTK::Pi / 2, SimTK::Pi, 0));
      // Known working
      imuPlacer.set_sensor_to_opensim_rotations(
          SimTK::Vec3(-SimTK::Pi / 2, SimTK::Pi / 2, 0));

      imuPlacer.set_orientation_file_for_calibration(file.string());

      imuPlacer.set_model_file(modelSourcePath.string());

      const std::string scaledOutputModelFilePrefix =
          outputFilePrefix + sep + imuSuffix;
      const std::string scaledOutputModelFile =
          resultDir /
          (scaledOutputModelFilePrefix + modelSourcePath.extension().string());
      sync_out.println("Scaled Output Model File: ", scaledOutputModelFile);

      imuPlacer.set_output_model_file(scaledOutputModelFile);
      imuPlacer.run();

      // Fix bug for set_model_file
      OpenSim::IMUPlacer imuPlacer2;
      imuPlacer2.set_base_imu_label("pelvis_imu");
      imuPlacer2.set_base_heading_axis("-z");
      // 90 0 90
      // imuPlacer2.set_sensor_to_opensim_rotations(
      //     SimTK::Vec3(-SimTK::Pi / 2, SimTK::Pi, 0));
      // Known working
      imuPlacer2.set_sensor_to_opensim_rotations(
          SimTK::Vec3(-SimTK::Pi / 2, SimTK::Pi / 2, 0));

      // imuPlacer2.set_orientation_file_for_calibration(file.string());

      imuPlacer2.set_model_file(modelSourcePath.string());

      // Now Run with removed IMUs
      const std::string imu_removed_suffix = "femur_IMUs_removed";
      const std::string orientationRemovedIMUsFile =
          resultDir /
          (file.stem().string() + sep + imu_removed_suffix + file.extension().string());
      OpenSim::TimeSeriesTable_<SimTK::Quaternion> quatTable(file.string());
      quatTable.removeColumn("femur_r_imu");
      quatTable.removeColumn("femur_l_imu");
      OpenSim::STOFileAdapter_<SimTK::Quaternion>::write(
        quatTable, orientationRemovedIMUsFile);

      imuPlacer2.set_orientation_file_for_calibration(
          orientationRemovedIMUsFile);

      const std::string scaledOutputModelFileRemovedIMUs =
          resultDir / (scaledOutputModelFilePrefix + sep + imu_removed_suffix +
                       modelSourcePath.extension().string());
      sync_out.println("Scaled Output Model File Removed IMUs: ",
                       scaledOutputModelFileRemovedIMUs);

      imuPlacer2.set_output_model_file(scaledOutputModelFileRemovedIMUs);
      imuPlacer2.run();

    } else {
      sync_out.println("Model Path doesn't exist: ", modelSourcePath);
    }
  } catch (const std::exception &e) {
    // Catching standard exceptions
    sync_out.println("Error in processing: ", e.what());
  } catch (...) {
    sync_out.println("Error in processing File: ", file.string());
  }
  sync_out.println("-------Finished Model Result Dir: ", resultDir.string(),
                   " File: ", file.stem().string());
}

void collectFiles(const std::filesystem::path &directory,
                  std::vector<std::filesystem::path> &files) {
  // Check if the path is a directory
  if (std::filesystem::is_directory(directory)) {
    // Iterate through the directory
    for (const auto &entry : std::filesystem::directory_iterator(directory)) {
      if (std::filesystem::is_directory(entry)) {
        // Recursively call collectFiles for subdirectories
        collectFiles(entry.path(), files);
      } else if (std::filesystem::is_regular_file(entry)) {
        // Add the file to the vector
        files.push_back(entry.path());
      }
    }
  }
}

// Function to filter files based on specific criteria
void filterFiles(const std::vector<std::filesystem::path> &allFiles,
                 std::vector<std::filesystem::path> &filteredFiles,
                 const std::vector<std::string> &includedParticipants) {
  for (const auto &path : allFiles) {
    std::string filename = path.stem().string();

    // Get the last two parent directories
    const std::filesystem::path firstParent = path.parent_path();
    const std::filesystem::path secondParent = firstParent.parent_path();

    const std::string participantId = secondParent.filename().string();

    // Check if the participant ID is in the included list
    const auto it = std::find(includedParticipants.begin(),
                              includedParticipants.end(), participantId);
    const bool participantIncluded = it != includedParticipants.end();

    // Check the file extension and naming conditions
    if (path.extension() == ".sto" &&
        (filename.rfind("data_l_", 0) == 0 ||
         filename.rfind("data_r_", 0) == 0) &&
        filename.ends_with("_orientations") && participantIncluded) {
      // Add the file to the filtered vector if all conditions are met
      filteredFiles.push_back(path);
    }
  }
}

// Function to create the required directory structure
void createResultDirectory(const std::filesystem::path &filePath,
                           const std::filesystem::path &resultPath) {
  const std::filesystem::path firstParent = filePath.parent_path();
  const std::filesystem::path secondParent = firstParent.parent_path();

  // Construct the result directory path
  const std::filesystem::path resultDir =
      resultPath / secondParent.filename() / firstParent.filename();

  // Create the directory if it doesn't exist
  if (!std::filesystem::exists(resultDir)) {
    if (std::filesystem::create_directories(resultDir)) {
      sync_out.println("Directories created: ", resultDir);
    } else {
      sync_out.println("Failed to create directory: ", resultDir);
    }
  }
}

int main(int argc, char *argv[]) {
  std::chrono::steady_clock::time_point begin =
      std::chrono::steady_clock::now();
  if (argc < 4) {
    std::cerr << "Usage: " << argv[0]
              << " <directory_path> <models_path> <output_path>" << std::endl;
    return 1;
  }

  std::filesystem::path directoryPath = argv[1];
  if (!std::filesystem::exists(directoryPath) ||
      !std::filesystem::is_directory(directoryPath)) {
    std::cerr << "The provided path is not a valid directory" << directoryPath
              << std::endl;
    return 1;
  }
  std::filesystem::path modelsPath = argv[2];
  if (!std::filesystem::exists(modelsPath) ||
      !std::filesystem::is_directory(modelsPath)) {
    std::cerr << "The provided path is not a valid directory: " << modelsPath
              << std::endl;
    return 1;
  }

  std::filesystem::path outputPath = argv[3];
  if (!std::filesystem::exists(outputPath)) {
    // Create the directory
    if (std::filesystem::create_directories(outputPath)) {
      sync_out.println("Directories created: ", outputPath);
    } else {
      sync_out.println("Failed to create directory: ", outputPath);
    }
  }

  // Threading
  BS::thread_pool pool;
  sync_out.println("Thread Pool num threads: ", pool.get_thread_count());

  // Start the recursive file collection
  std::vector<std::filesystem::path> allFiles;
  collectFiles(directoryPath, allFiles);

  // Filter the collected files based on the criteria
  std::vector<std::filesystem::path> filteredFiles;
  filterFiles(allFiles, filteredFiles, includedParticipants);

  // Create directories for each filtered file
  for (const auto &file : filteredFiles) {
    createResultDirectory(file, outputPath);
  }

  // Create configuration for running IK
  OpenSim::IO::SetDigitsPad(4);

  // Generate subject and trial specific models
  for (const auto &file : filteredFiles) {
    for (const auto &m : baseModels) {
      // Find the Model
      const std::filesystem::path firstParent = file.parent_path();
      const std::filesystem::path secondParent = firstParent.parent_path();
      const std::filesystem::path modelPath =
          modelsPath / secondParent.filename();
      const std::filesystem::path resultDir =
          outputPath / secondParent.filename() / firstParent.filename() / "";
      const ConfigType newConfig = {"", (modelPath / m).string()};
      pool.detach_task([file, resultDir, newConfig] {
        process(file, resultDir, newConfig);
      });
    }
  }
  // Wait for all tasks to finish
  pool.wait();

  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  const double runtime =
      std::chrono::duration_cast<std::chrono::microseconds>(end - begin)
          .count();
  sync_out.println("Runtime = ", runtime, " [µs]");
  sync_out.println("Finished Running without Error!");
  return 0;
}