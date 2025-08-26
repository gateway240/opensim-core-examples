cmake_minimum_required(VERSION 3.25)
project(HopperDeviceExample)

# Settings.
# ---------
set(TARGET exampleHopperDevice CACHE TYPE STRING)

# OpenSim uses C++17 language features.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -march=native")

# Find and hook up to OpenSim.
# ----------------------------
find_package(OpenSim REQUIRED PATHS "${OPENSIM_INSTALL_DIR}")

# Configure this project.
# -----------------------
file(GLOB SOURCE_FILES *.h *.cpp)

add_executable(${TARGET} ${SOURCE_FILES})

target_link_libraries(${TARGET} ${OpenSim_LIBRARIES})