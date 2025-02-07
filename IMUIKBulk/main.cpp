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
#include <iterator> // For std::back_inserter
#include <memory>
#include <optional>
#include <regex>
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
typedef std::pair<OpenSim::OrientationWeightSet, std::string> ConfigType;

const std::vector<OpenSim::OrientationWeightSet> orientationWeightSets = {
    OpenSim::OrientationWeightSet("setup_OrientationWeightSet_uniform.xml"),
    OpenSim::OrientationWeightSet(
        "setup_OrientationWeightSet_pelvis_tibia_calcn.xml"),
    // OpenSim::OrientationWeightSet(
    //     "setup_OrientationWeightSet_pelvis_tibia.xml"),
    // OpenSim::OrientationWeightSet(
    //     "setup_OrientationWeightSet_pelvis_calcn.xml"),
    // OpenSim::OrientationWeightSet("setup_OrientationWeightSet_pelvis.xml"),
    // OpenSim::OrientationWeightSet(
    //     "setup_OrientationWeightSet_tibia_calcn.xml")
        };

const std::vector<std::string> baseModels = {
    "kg_gait2392_thelen2003muscle_scaled.osim",
    // "kg_Rajagopal2016_scaled.osim",
        // "kg_KUOPIO_base_R_pelvis_relaxed_scaled_only.osim"
    };

// Simple test case
const bool allParticipants = true;
const std::vector<std::string> includedParticipants = {"40"};

// All trials with no invalid trials
// const std::vector<std::string> includedParticipants = {
//     "08", "09", "12", "17", "19", "21", "24",
//     "28", "31", "34", "36", "38", "40", "41"};

const std::string imu_removed_suffix = "";
const std::string outputBasePrefix = "kg";
const std::string imuSuffix = "and_IMUs";

const std::string sep = "_";

void process(const std::filesystem::path &file,
             const std::filesystem::path &resultDir, const ConfigType &c) {
  sync_out.println("---Starting IK Processing: ", file.string());
  try {
    const OpenSim::OrientationWeightSet weightSet = c.first;
    const std::string weightSetName = weightSet.getName();

    const std::filesystem::path modelSourcePath = c.second;
    const std::string modelSourceStem = modelSourcePath.stem().string();

    const std::string outputFilePrefix = modelSourceStem + sep + weightSetName;

    sync_out.println("Model Path: ", modelSourcePath.string(),
                     " Weight Set Name: ", weightSetName);

    const std::string outputSuffix = "imu_ik_output";

    const std::filesystem::path outputMotionFile =
        resultDir / (outputFilePrefix + sep + outputSuffix + ".mot");

    if (std::filesystem::exists(modelSourcePath)) {
      OpenSim::IMUInverseKinematicsTool imuIk;
      imuIk.setName(outputFilePrefix);

      imuIk.set_accuracy(9.9999999999999995e-07);

      const OpenSim::Array<double> range{SimTK::Infinity, 2};
      // Make range -Infinity to Infinity unless limited by data
      range[0] = 0.0;
      imuIk.set_time_range(range);

      // This is the rotation for the kuopio gait dataset
      const SimTK::Vec3 rotations(-SimTK::Pi / 2, 0, 0);
      imuIk.set_sensor_to_opensim_rotations(rotations);
      imuIk.set_model_file(modelSourcePath.string());
      imuIk.set_orientations_file(file.string());
      imuIk.set_results_directory(resultDir);
      imuIk.set_output_motion_file(outputMotionFile.string());
      imuIk.set_orientation_weights(weightSet);
      bool visualizeResults = false;
      imuIk.run(visualizeResults);
      imuIk.print((resultDir / (outputFilePrefix + sep + outputSuffix + ".xml"))
                      .string());
    } else {
      sync_out.println("Model Path doesn't exist: ", modelSourcePath);
    }
  } catch (const std::exception &e) {
    // Catching standard exceptions
    sync_out.println("Error in processing: ", e.what());
  } catch (...) {
    sync_out.println("Error in processing File: ", file.string());
  }
  sync_out.println("-------Finished IK Result Dir: ", resultDir.string(),
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
    const bool participantIncluded = (it != includedParticipants.end() || allParticipants);

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

std::optional<std::smatch> matchesPattern(const std::string &filename) {
  // Define the regex pattern for "<r or l>_<comf fast or slow>_<two digit
  // number>"
  std::regex pattern(R"([rl]_(fast|slow|comf)_(\d{2}))");
  std::smatch match;
  if (std::regex_search(filename, match, pattern)) {
    return match; // Return the matched groups
  }
  return std::nullopt; // Return an empty optional if no match is found
}

std::optional<std::filesystem::path>
findFirstFile(const std::filesystem::path &directory,
              const std::string &modelSearchString,
              const std::string &imuRemovedSearchString,
              const std::string &trialSearchString) {
  for (const auto &entry : std::filesystem::directory_iterator(directory)) {
    if (entry.is_regular_file()) {
      const auto &path = entry.path();
      const auto &filename = path.filename().string();

      if (path.extension() == ".osim" &&
          // file.string().find(modelSearchString) != std::string::npos &&
          path.string().find(modelSearchString) != std::string::npos &&
          path.string().find(imuRemovedSearchString) != std::string::npos &&
          path.string().find(trialSearchString) != std::string::npos) {
        // std::cout << "file: " << file.string() << std::endl;
        // std::cout << "model: " << modelSearchString << std::endl;
        // std::cout << "path: " << path << std::endl;
        // std::cout << std::endl;
        // std::cout << path << " search: " << searchString << std::endl;
        return path;
      }
    }
  }
  return std::nullopt; // Return an empty optional if no match is found
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
  const int max_threads = 64;
  const int num_threads = std::thread::hardware_concurrency() > max_threads
                              ? max_threads
                              : std::thread::hardware_concurrency();
  BS::thread_pool pool(num_threads);
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

  std::vector<ConfigType> config;

  // Create every permutation of OrientationWeightSet and base model
  std::for_each(orientationWeightSets.begin(), orientationWeightSets.end(),
                [&](const OpenSim::OrientationWeightSet &weightSet) {
                  std::transform(baseModels.begin(), baseModels.end(),
                                 std::back_inserter(config),
                                 [&](const std::string &model) {
                                   return std::make_pair(weightSet, model);
                                 });
                });

  // Run IK on all permutations
  for (const auto &file : filteredFiles) {
    for (const auto &c : config) {
      // std::cout << "File: " << file << std::endl;
      // Find the Model
      const std::filesystem::path firstParent = file.parent_path();
      const std::filesystem::path secondParent = firstParent.parent_path();
      const std::filesystem::path resultDir =
          outputPath / secondParent.filename() / firstParent.filename() / "";
      const std::filesystem::path fileNameBaseModel = c.second;
      auto match = matchesPattern(file.string());
      if (match) {
        const std::string trial = match.value()[0];
        std::cout << "Full match: " << trial << " result dir: " << resultDir << std::endl;
        const auto result =
            findFirstFile(resultDir, fileNameBaseModel.stem(),
                          imu_removed_suffix, trial);
        if (result) {
          const std::string modelPath = *result;
          std::cout << "Model path: " << modelPath << std::endl;
          const ConfigType newConfig = {c.first, modelPath};
          pool.detach_task([file, resultDir, newConfig] {
            process(file, resultDir, newConfig);
          });
        }
      }
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