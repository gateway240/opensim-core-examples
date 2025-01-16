cmake_minimum_required(VERSION 3.22)

project(Opensim_Examples)

# Settings.
# ---------
set(TARGET "main" CACHE STRING "main")

# OpenSim uses C++11 language features.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -march=native")

# Find and hook up to OpenSim.
# ----------------------------
set(OpenSim_DIR "~/opensim-core/cmake")
find_package(OpenSim REQUIRED PATHS "${OPENSIM_INSTALL_DIR}")

# Thread Pool Lib
# ----------------------------
if(MSVC)
    add_compile_options(/permissive- /Zc:__cplusplus)
endif()
set(CPM_DOWNLOAD_LOCATION ${CMAKE_BINARY_DIR}/CPM.cmake)
if(NOT(EXISTS ${CPM_DOWNLOAD_LOCATION}))
    file(DOWNLOAD https://github.com/cpm-cmake/CPM.cmake/releases/latest/download/CPM.cmake ${CPM_DOWNLOAD_LOCATION})
endif()
include(${CPM_DOWNLOAD_LOCATION})

CPMAddPackage("gh:bshoshany/thread-pool@5.0.0")
add_library(BS_thread_pool INTERFACE)
target_include_directories(BS_thread_pool INTERFACE ${${CPM_LAST_PACKAGE_NAME}_SOURCE_DIR}/include)


# Configure this project.
# -----------------------
file(GLOB SOURCE_FILES *.h *.cpp)

add_executable(${TARGET} ${SOURCE_FILES})

target_link_libraries(${TARGET} ${OpenSim_LIBRARIES}  BS_thread_pool)

# This block symlinks the data files from data additional files into the running directory
set(DATA_DIR "${CMAKE_SOURCE_DIR}/data")
file(GLOB FILES "${DATA_DIR}/*")
foreach(FILE ${FILES})
    get_filename_component(FILENAME ${FILE} NAME)
    add_custom_command(TARGET ${TARGET} PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E create_symlink
                    ${FILE} $<TARGET_FILE_DIR:${TARGET}>/${FILENAME})
endforeach()