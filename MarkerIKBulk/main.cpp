// INCLUDES
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/TRCFileAdapter.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Tools/InverseKinematicsTool.h>


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

const std::vector<std::pair<std::string, std::string>> config = {
    // {"kuopio_base_IK_Tasks_uniform.xml", "kg_gait2392_thelen2003muscle_scaled_and_markerIK_and_IMUs.osim"},
    {"kg_IK_Tasks_uniform.xml", "kg_gait2392_thelen2003muscle_scaled_and_markerIK_and_IMUs.osim"},
    {"kg_IK_Tasks_uniform.xml", "kg_Rajagopal2016_scaled_and_markerIK_and_IMUs.osim"}
        // {"kg_IK_Tasks_uniform.xml", "kg_Rajagopal2016_scaled_only.osim"},
    };


const std::string outputBasePrefix = "kg_result";
const std::string fileNameSetupInverseKinematics = "setup_InverseKinematics.xml";

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

void process(const std::filesystem::path &sourceDir,
             const std::filesystem::path &modelsDir,
             const std::filesystem::path &resultDir,
             const std::filesystem::path &file,
             const std::pair<std::string, std::string> &config) {
  std::cout << "---Starting Processing: " << sourceDir << " file: " << file
            << " Model: " << config.second << std::endl;
  try {
    // OpenSim::IO::SetDigitsPad(4);


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
    const std::filesystem::path markerFilePath =
        resultDir / rotatedFilename;
    // std::cout << markerFilePath.string() << std::endl;
    
    // Write the rotated file
    std::filesystem::path newDirectory = resultDir;

    const std::string markerFileName = markerFilePath.string();

    trcfileadapter.write(table, markerFileName);

    // Find the Model
    const std::filesystem::path firstParent = sourceDir.parent_path();

    const std::string fileNameIKTaskSet = config.first;
    const std::string fileNameBaseModel = config.second;
    const std::filesystem::path modelSourcePath =
        modelsDir / firstParent.filename() / fileNameBaseModel;
    const std::string modelSourceStem = modelSourcePath.stem().string();
    // std::cout << "ModelsDir: " << modelsDir << std::endl;
    // std::cout << "First Parent: " << firstParent << " Second Parent: " << secondParent << std::endl;
    std::cout << "Model path: " << modelSourcePath << std::endl;
    
    const std::string outputMotionFileName =
        outputBasePrefix + sep + markerFilePath.stem().string() + sep + modelSourceStem + sep + "marker_ik_output.mot";

    const std::filesystem::path resultsFirstParent = resultDir.parent_path();
    const std::filesystem::path resultsSecondParent = resultsFirstParent.parent_path();


    if (std::filesystem::exists(modelSourcePath)) {
        OpenSim::InverseKinematicsTool ik((resultDir / fileNameSetupInverseKinematics).string());
        // OpenSim::Model mdl(modelSourcePath.string());
        // mdl.initSystem();
        // ik.setModel(mdl);
        ik.set_model_file(modelSourcePath.string());

        ik.set_marker_file(markerFileName);
        // ik.setMarkerDataFileName(markerFileName);
        ik.set_output_motion_file(outputMotionFileName);
        
        bool ikSuccess = false;
        double startTime = ik.getStartTime();
        double endTime = ik.getEndTime();
        const double timeIncrement = 0.1;
        // std::cout << "Start Time: " << startTime << " End Time: " << endTime << std::endl;
        do {
            std::cerr << "Running IK! Trying with start time: " << startTime << std::endl; 
            try {
                ikSuccess = ik.run();
            } catch ( ... ) {
                std::cerr << "IK Failed! Incrementing and trying again!" << std::endl;
            } 
            startTime += timeIncrement;
            ik.setStartTime(startTime);
        } while (!ikSuccess && startTime < endTime);
  
    }

  } catch (const std::exception &e) {
    // Catching standard exceptions
    std::cerr << "Error in processing: " << e.what() << std::endl;
  } catch (...) {
    std::cout << "Error in processing Dir: " << sourceDir << std::endl;
  }

  std::cout << "-------Finished Result Dir: " << resultDir << " File: " << file.stem() << std::endl;
}

void preProcessDirectory(const std::filesystem::path &dirPath,
                      const std::filesystem::path &modelsPath,
                      const std::filesystem::path &resultPath,
                      const std::pair<std::string, std::string> &c) {

  // Iterate through the directory
  for (const auto &entry : std::filesystem::directory_iterator(dirPath)) {
    if (entry.is_directory()) {
      // Recursively process subdirectory
      preProcessDirectory(entry.path(), modelsPath, resultPath, c);
    } else if (entry.is_regular_file()) {
      // Check if the file has a .mat extension
      // std::cout << entry.path().filename() << std::endl;
      std::filesystem::path path = entry.path();
      // Get the filename as a string
      std::string filename = path.filename().string();
      // Only files that end in .trc and start with either l_ or r_
      if (path.extension() == ".trc" && (filename.rfind("l_", 0) == 0 || filename.rfind("r_", 0) == 0)) {
        // Create a corresponding text file
        
        // Get the last two parent directories
        const std::filesystem::path firstParent = path.parent_path();
        const std::filesystem::path secondParent = firstParent.parent_path();

        const std::filesystem::path resultDir =
            resultPath / secondParent.filename() / firstParent.filename() / "";

        std::filesystem::path newDirectory = resultDir.parent_path();
        if (!std::filesystem::exists(newDirectory)) {
          // Create the directory
          if (std::filesystem::create_directories(newDirectory)) {
            std::cout << "Directories created: " << newDirectory << std::endl;
          } else {
            std::cerr << "Failed to create directory: " << newDirectory
                      << std::endl;
          }
        }
        const std::string fileNameIKTaskSet = c.first;
        const std::filesystem::path ikTaskSetSourcePath(fileNameIKTaskSet);
        const std::filesystem::path ikTaskSetDestinationPath = resultDir /  fileNameIKTaskSet;

        const std::string fileNameSetupIK = fileNameSetupInverseKinematics;
        const std::filesystem::path setupIKSourcePath(fileNameSetupIK);
        const std::filesystem::path setupIKDestinationPath = resultDir / fileNameSetupIK;
        try {
            // Copy the file to the destination directory
            std::filesystem::copy_file(ikTaskSetSourcePath, ikTaskSetDestinationPath, std::filesystem::copy_options::overwrite_existing);
            std::filesystem::copy_file(setupIKSourcePath, setupIKDestinationPath, std::filesystem::copy_options::overwrite_existing);
        } catch (const std::filesystem::filesystem_error& e) {
            std::cerr << "Error copying file: " << e.what() << std::endl;
        }
      }
    }
  }
  return;
}

void processDirectory(const std::filesystem::path &dirPath,
                      const std::filesystem::path &modelsPath,
                      const std::filesystem::path &resultPath,
                      const std::pair<std::string, std::string> &c) {

  std::vector<std::thread> threads;
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
      std::string filename = path.filename().string();
      // Only files that end in .trc and start with either l_ or r_
      if (path.extension() == ".trc" && (filename.rfind("l_", 0) == 0 || filename.rfind("r_", 0) == 0)) {
        // Create a corresponding text file
        
        // Get the last two parent directories
        const std::filesystem::path firstParent = path.parent_path();
        const std::filesystem::path secondParent = firstParent.parent_path();

        const std::filesystem::path resultDir =
            resultPath / secondParent.filename() / firstParent.filename() / "";

        threads.emplace_back(process, dirPath, modelsPath, resultDir, path, c);

      }
    }
  }
  // Join all threads
  for (auto &thread : threads) {
    if (thread.joinable()) {
      thread.join();
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
      std::cout << "Directories created: " << outputPath << std::endl;
    } else {
      std::cerr << "Failed to create directory: " << outputPath << std::endl;
    }
  }

  for (const auto &c : config) {
    // const std::string fileNameIKTaskSet = c.first;
    // const std::filesystem::path ikTaskSetSourcePath(fileNameIKTaskSet);
    // const std::filesystem::path ikTaskSetDestinationPath = outputPath /  fileNameIKTaskSet;

    // const std::string fileNameSetupIK = fileNameSetupInverseKinematics;
    // const std::filesystem::path setupIKSourcePath(fileNameSetupIK);
    // const std::filesystem::path setupIKDestinationPath = outputPath / fileNameSetupIK;
    try {
        // Copy the file to the destination directory
        // std::filesystem::copy_file(ikTaskSetSourcePath, ikTaskSetDestinationPath, std::filesystem::copy_options::overwrite_existing);
        // std::filesystem::copy_file(setupIKSourcePath, setupIKDestinationPath, std::filesystem::copy_options::overwrite_existing);
        // Run processing if the copies didn't fail
        preProcessDirectory(directoryPath, modelsPath, outputPath, c);
        processDirectory(directoryPath, modelsPath, outputPath, c);
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Error copying file: " << e.what() << std::endl;
    }
  }

  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  std::cout << "Runtime = "
            << std::chrono::duration_cast<std::chrono::microseconds>(end -
                                                                     begin)
                   .count()
            << "[µs]" << std::endl;
  std::cout << "Finished Running without Error!" << std::endl;
  return 0;
}