cmake_minimum_required(VERSION 3.22)

project(Opensim_Examples)

# Settings.
# ---------
set(TARGET "main" CACHE STRING "main")

# OpenSim uses C++11 language features.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -march=native")

# Find and hook up to OpenSim.
# ----------------------------
find_package(OpenSim REQUIRED PATHS "${OPENSIM_INSTALL_DIR}")

# Thread Pool Lib
# ----------------------------
include(FetchContent)
set(FETCHCONTENT_UPDATES_DISCONNECTED OFF)
FetchContent_Declare(
    bshoshany_thread_pool
    GIT_REPOSITORY https://github.com/bshoshany/thread-pool.git
    GIT_TAG v5.0.0
)
FetchContent_MakeAvailable(bshoshany_thread_pool)
add_library(BS_thread_pool INTERFACE)
target_include_directories(BS_thread_pool INTERFACE ${bshoshany_thread_pool_SOURCE_DIR}/include)


# Configure this project.
# -----------------------
file(GLOB SOURCE_FILES *.h *.cpp)

add_executable(${TARGET} ${SOURCE_FILES})

target_link_libraries(${TARGET} PRIVATE ${OpenSim_LIBRARIES}  BS_thread_pool)

# This block symlinks the data files from data additional files into the running directory
set(DATA_DIR "${CMAKE_SOURCE_DIR}/data")
file(GLOB FILES "${DATA_DIR}/*")
foreach(FILE ${FILES})
    get_filename_component(FILENAME ${FILE} NAME)
    add_custom_command(TARGET ${TARGET} PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E create_symlink
                    ${FILE} $<TARGET_FILE_DIR:${TARGET}>/${FILENAME})
endforeach()