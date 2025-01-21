// INCLUDES
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/TRCFileAdapter.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Tools/InverseKinematicsTool.h>

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
#include <vector>

// Logging output
const int64_t time_now =
    std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch())
        .count();
std::ofstream log_file("task-" + std::to_string(time_now) + ".log");
BS::synced_stream sync_out(std::cout, log_file);

typedef std::pair<std::string, std::string> ConfigType;
// All trials with no invalid trials
const std::vector<std::string> includedParticipants = {
    "08", "09", "12", "17", "19", "21", "24",
    "28", "31", "34", "36", "38", "40", "41"};

const std::vector<ConfigType> config = {
    // {"kuopio_base_IK_Tasks_uniform.xml",
    // "kg_gait2392_thelen2003muscle_scaled_and_markerIK_and_IMUs.osim"},
    {"kg_IK_Tasks_uniform.xml",
     "kg_gait2392_thelen2003muscle_scaled_and_markerIK.osim"},
    // {"kg_IK_Tasks_uniform.xml", "kg_Rajagopal2016_scaled_and_markerIK.osim"}
    // {"kg_IK_Tasks_uniform.xml", "kg_Rajagopal2016_scaled_only.osim"},
};

const std::string outputBasePrefix = "kg_result";
const std::string fileNameSetupInverseKinematics =
    "setup_InverseKinematics.xml";

const std::string sep = "_";
// Rotation from marker space to OpenSim space (y is up)
// This is the rotation for the kuopio gait dataset
const SimTK::Vec3 rotations(-SimTK::Pi / 2, SimTK::Pi / 2, 0);

// Function to rotate a table of Vec3 elements
void rotateMarkerTable(OpenSim::TimeSeriesTableVec3 &table,
                       const SimTK::Rotation_<double> &rotationMatrix) {
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

void process(const std::filesystem::path &file,
             const std::filesystem::path &resultDir, const ConfigType &c) {
  sync_out.println("---Starting Marker IK Processing: ", file.string());
  try {
    OpenSim::IO::SetDigitsPad(4);

    const std::filesystem::path sourceTrcFile = file;

    // ROTATE the marker table so the orientation is correct
    OpenSim::TRCFileAdapter trcfileadapter{};
    OpenSim::TimeSeriesTableVec3 table{sourceTrcFile.string()};

    const SimTK::Rotation sensorToOpenSim = SimTK::Rotation(
        SimTK::BodyOrSpaceType::SpaceRotationSequence, rotations[0],
        SimTK::XAxis, rotations[1], SimTK::YAxis, rotations[2], SimTK::ZAxis);
    rotateMarkerTable(table, sensorToOpenSim);

    // Get the filename without extension
    const std::string rotatedFilename = sourceTrcFile.stem().string() +
                                        "_rotated" +
                                        sourceTrcFile.extension().string();
    const std::filesystem::path markerFilePath = resultDir / rotatedFilename;
    // std::cout << markerFilePath.string() << std::endl;

    // Write the rotated file
    const std::string markerFileName = markerFilePath.string();

    trcfileadapter.write(table, markerFileName);

    // Find the Model
    // const std::filesystem::path firstParent = sourceDir.parent_path();

    const std::string fileNameIKTaskSet = c.first;
    const std::filesystem::path modelSourcePath = c.second;
    const std::string modelSourceStem = modelSourcePath.stem().string();
    // std::cout << "ModelsDir: " << modelsDir << std::endl;
    // std::cout << "First Parent: " << firstParent << " Second Parent: " <<
    // secondParent << std::endl;
    sync_out.println("Model Path: ", modelSourcePath.string());
    // std::cout << "Model path: " << modelSourcePath << std::endl;

    const std::string outputFilePrefix = outputBasePrefix + sep +
                                         markerFilePath.stem().string() + sep +
                                         modelSourceStem;
    const std::filesystem::path outputMotionFile =
        resultDir / (outputFilePrefix + sep + "marker_ik_output.mot");

    const std::filesystem::path resultsFirstParent = resultDir.parent_path();
    const std::filesystem::path resultsSecondParent =
        resultsFirstParent.parent_path();

    if (std::filesystem::exists(modelSourcePath)) {
      OpenSim::InverseKinematicsTool ik(
          (resultDir / fileNameSetupInverseKinematics).string());
      ik.setName(outputFilePrefix);
      // OpenSim::Model mdl(modelSourcePath.string());
      // mdl.initSystem();
      // ik.setModel(mdl);
      ik.set_report_marker_locations(false);
      ik.set_model_file(modelSourcePath.string());

      ik.set_marker_file((resultDir / markerFileName).string());
      // ik.setMarkerDataFileName(markerFileName);
      ik.set_output_motion_file(outputMotionFile.string());

      bool ikSuccess = false;
      double startTime = ik.getStartTime();
      double endTime = ik.getEndTime();
      const double timeIncrement = 0.1;
      // std::cout << "Start Time: " << startTime << " End Time: " << endTime <<
      // std::endl;
      do {
        sync_out.println("Running IK! Trying with start time: ", startTime);
        try {
          ikSuccess = ik.run();
        } catch (...) {
          sync_out.println("IK Failed! Incrementing and trying again!");
        }
        startTime += timeIncrement;
        ik.setStartTime(startTime);
      } while (!ikSuccess && startTime < endTime);
      ik.print((resultDir / (outputFilePrefix + sep + "marker_ik_output.xml"))
                   .string());
    }

  } catch (const std::exception &e) {
    // Catching standard exceptions
    sync_out.println("Error in processing: ", e.what());
  } catch (...) {
    sync_out.println("Error in processing File: ", file.string());
  }
  sync_out.println("-------Finished Result Dir: ", resultDir.string(),
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
    if (path.extension() == ".trc" &&
        (filename.rfind("l_", 0) == 0 || filename.rfind("r_", 0) == 0) &&
        participantIncluded) {
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
  for (const auto &file : filteredFiles) {
    for (const auto &c : config) {
      const std::filesystem::path firstParent = file.parent_path();
      const std::filesystem::path secondParent = firstParent.parent_path();
      const std::filesystem::path resultDir =
          outputPath / secondParent.filename() / firstParent.filename() / "";
      const std::string fileNameBaseModel = c.second;
      const std::filesystem::path modelSourcePath =
          modelsPath / secondParent.filename() / fileNameBaseModel;

      const std::string fileNameIKTaskSet = c.first;
      const std::filesystem::path ikTaskSetSourcePath(fileNameIKTaskSet);
      const std::filesystem::path ikTaskSetDestinationPath =
          resultDir / fileNameIKTaskSet;

      const std::string fileNameSetupIK = fileNameSetupInverseKinematics;
      const std::filesystem::path setupIKSourcePath(fileNameSetupIK);
      const std::filesystem::path setupIKDestinationPath =
          resultDir / fileNameSetupIK;
      try {
        // Copy the file to the destination directory
        std::filesystem::copy_file(
            ikTaskSetSourcePath, ikTaskSetDestinationPath,
            std::filesystem::copy_options::update_existing);
        std::filesystem::copy_file(
            setupIKSourcePath, setupIKDestinationPath,
            std::filesystem::copy_options::update_existing);

        const ConfigType newConfig = {c.first, (modelSourcePath).string()};
        pool.detach_task([file, resultDir, newConfig] {
          process(file, resultDir, newConfig);
        });
      } catch (const std::filesystem::filesystem_error &e) {
        sync_out.println("Error in copying File: ", e.what());
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