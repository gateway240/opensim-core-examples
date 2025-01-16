// INCLUDES
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/TRCFileAdapter.h>
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
#include <memory>
#include <string>
#include <thread>
#include <vector>

std::ofstream log_file("task.log");
BS::synced_stream sync_out(std::cout, log_file);
const int max_threads = 48;
const int num_threads = std::thread::hardware_concurrency() > max_threads
                            ? max_threads
                            : std::thread::hardware_concurrency();
BS::thread_pool pool(num_threads);

// All trials with no invalid trials
const std::vector<std::string> includedParticipants = {
    "08", "09", "12", "17", "19", "21", "24",
    "28", "31", "34", "36", "38", "40", "41"};

const std::vector<std::pair<std::string, std::string>> config = {
    // {"kuopio_base_IK_Tasks_uniform.xml",
    // "kg_gait2392_thelen2003muscle_scaled_and_markerIK_and_IMUs.osim"},
    {"kg", "kg_gait2392_thelen2003muscle_scaled_only.osim"},
    {"kg", "kg_Rajagopal2016_scaled_only.osim"}
    // {"kg_IK_Tasks_uniform.xml", "kg_Rajagopal2016_scaled_only.osim"},
};

const std::string outputBasePrefix = "kg_result";

const std::string sep = "_";

void process(const std::filesystem::path &sourceDir,
             const std::filesystem::path &modelsDir,
             const std::filesystem::path &resultDir,
             const std::filesystem::path &file,
             const std::pair<std::string, std::string> &c) {
  sync_out.println("---Starting Processing: ", sourceDir.string(),
                   " file: ", file.string(), " Model: ", c.second);
  try {
    OpenSim::IO::SetDigitsPad(4);

    // Find the Model
    const std::filesystem::path firstParent = sourceDir.parent_path();

    const std::string fileNameIKTaskSet = c.first;
    const std::string fileNameBaseModel = c.second;
    const std::filesystem::path modelSourcePath =
        modelsDir / firstParent.filename() / fileNameBaseModel;
    const std::string modelSourceStem = modelSourcePath.stem().string();
    // std::cout << "ModelsDir: " << modelsDir << std::endl;
    // std::cout << "First Parent: " << firstParent << " Second Parent: " <<
    // secondParent << std::endl;
    sync_out.println("Model Path: ", modelSourcePath.string());
    // std::cout << "Model path: " << modelSourcePath << std::endl;

    const std::string outputSuffix = "imu_ik_output";
    const std::string outputFilePrefix =
        outputBasePrefix + sep + file.stem().string() + sep + modelSourceStem;
    const std::filesystem::path outputMotionFile =
        resultDir / (outputFilePrefix + sep + outputSuffix + ".mot");

    const std::filesystem::path resultsFirstParent = resultDir.parent_path();
    const std::filesystem::path resultsSecondParent =
        resultsFirstParent.parent_path();

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

      // Scaled Output Model
      imuPlacer.set_model_file(modelSourcePath.string());

      const std::string scaledOutputModelFile =
          resultDir / (outputFilePrefix + sep + "and_IMUs" +
                       modelSourcePath.extension().string());
      sync_out.println("Scaled Output Model File: ", scaledOutputModelFile);

      imuPlacer.set_output_model_file(scaledOutputModelFile);
      imuPlacer.run();

      OpenSim::IMUInverseKinematicsTool imuIk;
      imuIk.setName(outputFilePrefix);

      imuIk.set_accuracy(9.9999999999999995e-07);

      const OpenSim::Array<double> range{SimTK::Infinity, 2};
      // Make range -Infinity to Infinity unless limited by data
      range[0] = 4.0;
      imuIk.set_time_range(range);

      // This is the rotation for the kuopio gait dataset
      const SimTK::Vec3 rotations(-SimTK::Pi / 2, 0, 0);
      imuIk.set_sensor_to_opensim_rotations(rotations);
      imuIk.set_model_file(scaledOutputModelFile);
      imuIk.set_orientations_file(file.string());
      imuIk.set_results_directory(resultDir);
      imuIk.set_output_motion_file(outputMotionFile.string());
      bool visualizeResults = false;
      imuIk.run(visualizeResults);
      imuIk.print((resultDir / (outputFilePrefix + sep + outputSuffix + ".xml"))
                      .string());
    }

  } catch (const std::exception &e) {
    // Catching standard exceptions
    sync_out.println("Error in processing: ", e.what());
  } catch (...) {
    sync_out.println("Error in processing Dir: ", sourceDir.string());
  }
  sync_out.println("-------Finished Result Dir: ", resultDir.string(),
                   " File: ", file.stem().string());
}

void processDirectory(const std::filesystem::path &dirPath,
                      const std::filesystem::path &modelsPath,
                      const std::filesystem::path &resultPath,
                      const std::pair<std::string, std::string> &c) {

  // Iterate through the directory
  for (const auto &entry : std::filesystem::directory_iterator(dirPath)) {
    if (entry.is_directory()) {
      // Recursively process subdirectory
      processDirectory(entry.path(), modelsPath, resultPath, c);
    } else if (entry.is_regular_file()) {
      // Check if the file has a .mat extension
      // std::cout << entry.path().filename() << std::endl;
      std::filesystem::path path = entry.path();
      // Get the filename as a string
      std::string filename = path.stem().string();

      // Get the last two parent directories
      const std::filesystem::path firstParent = path.parent_path();
      const std::filesystem::path secondParent = firstParent.parent_path();

      const std::string participantId = secondParent.filename();
      // Use std::find to check if the string is in the list
      const auto it = std::find(includedParticipants.begin(),
                                includedParticipants.end(), participantId);
      const bool participantIncluded = it != includedParticipants.end();
      // Only files that end in .trc and start with either l_ or r_
      if (path.extension() == ".sto" &&
          (filename.rfind("data_l_", 0) == 0 ||
           filename.rfind("data_r_", 0) == 0) &&
          filename.ends_with("_orientations") && participantIncluded) {
        const std::filesystem::path resultDir =
            resultPath / secondParent.filename() / firstParent.filename() / "";

        std::filesystem::path newDirectory = resultDir.parent_path();
        if (!std::filesystem::exists(newDirectory)) {
          // Create the directory
          if (std::filesystem::create_directories(newDirectory)) {
            sync_out.println("Directories created: ", newDirectory);
          } else {
            sync_out.println("Failed to create directory: ", newDirectory);
          }
        }
        pool.detach_task([dirPath, modelsPath, resultDir, path, c] {
          process(dirPath, modelsPath, resultDir, path, c);
        });
      }
    }
  }
  return;
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
  sync_out.println("Thread Pool num threads: ", pool.get_thread_count());

  for (const auto &c : config) {
    try {
      processDirectory(directoryPath, modelsPath, outputPath, c);
    } catch (const std::filesystem::filesystem_error &e) {
      sync_out.println("Error Processing: ", e.what());
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