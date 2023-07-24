cmake_minimum_required (VERSION 3.14)
project (PROFIKUM VERSION 1.0.0)

cmake_policy(SET CMP0115 NEW)

add_executable(profikum src/main.cpp)
target_include_directories(profikum
  PRIVATE
  src
  resources/profikum_arduino
)
target_sources(profikum
  PRIVATE
  src/Profikum.cpp
  src/SerialConnection.cpp
)

# Copy script which is responsible to set device's IP address upon profinet controller request to build tree.
# Note that this is only necessary if one wants to run the build files. The install files search at the correct position.
file (COPY ${CMAKE_INSTALL_PREFIX}/bin/profipp_network_parameters DESTINATION ${PROFIKUM_BINARY_DIR}/ FILE_PERMISSIONS WORLD_EXECUTE)

target_compile_features(profikum PRIVATE cxx_std_17)

# Add thread support (needed by profi++)
find_package(Threads REQUIRED)
target_link_libraries(profikum INTERFACE Threads::Threads)

# Add profinet support via profipp v1.0 or higher
find_package(profipp 1.0 REQUIRED)
target_link_libraries(profikum PUBLIC profipp)

# Install the example application
install (TARGETS profikum DESTINATION bin)
# Also install any additional files which might be required due to anything compiler specific
include (InstallRequiredSystemLibraries)
