####################################################################################################
# CMakeLists file for SIDOsc
# Generated by Yigit Kolat
####################################################################################################

# Basic project config
cmake_minimum_required(VERSION 3.12)
set(project_name "SIDOsc")
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH})
set(CMAKE_CXX_STANDARD 17)

####################################################################################################
# Load modules
include(SuperColliderServerPlugin RESULT_VARIABLE server_plugin_found)
if(NOT server_plugin_found)
    message(FATAL_ERROR "Could not find server plugin functions module")
endif()

include(SuperColliderCompilerConfig RESULT_VARIABLE compiler_config_found)
if(NOT compiler_config_found)
    message(FATAL_ERROR "Could not find compiler config module")
endif()

# Set the SC_PATH explicitly
set(SC_PATH "/Users/hayakzan/Desktop/SC-source" CACHE PATH
    "Path to SuperCollider source. Relative paths are treated as relative to this script" FORCE)

sc_check_sc_path("${SC_PATH}")
message(STATUS "Found SuperCollider: ${SC_PATH}")

include("${SC_PATH}/SCVersion.txt")
message(STATUS "Building plugins for SuperCollider version: ${SC_VERSION}")

# Set project here to avoid SCVersion.txt clobbering our version info
project(${project_name})
sc_do_initial_compiler_config() # Do after setting project so compiler ID is available

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT OR NOT CMAKE_INSTALL_PREFIX)
    message(WARNING "No install prefix provided, defaulting to $BUILD_DIR/install")
    set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Install prefix" FORCE)
endif()

message(STATUS "Install directory set to: ${CMAKE_INSTALL_PREFIX}")

####################################################################################################
# Options
option(SUPERNOVA "Build plugins for supernova" ON)
option(SCSYNTH "Build plugins for scsynth" ON)
option(NATIVE "Optimize for native architecture" OFF)
option(STRICT "Use strict warning flags" OFF)
option(NOVA_SIMD "Build plugins with nova-simd support." ON)

####################################################################################################
# Include libraries

if (NOVA_SIMD)
    add_definitions(-DNOVA_SIMD)
    include_directories(${SC_PATH}/external_libraries/nova-simd)
endif()

####################################################################################################
# Integrate conversion of waveform .dat files to header files
####################################################################################################
find_package(Perl REQUIRED)

# List the raw waveform data files (adjust paths if necessary; here we assume they reside in resid/)
set(WAVE_DAT_FILES
    "${CMAKE_SOURCE_DIR}/resid/wave6581_PST.dat"
    "${CMAKE_SOURCE_DIR}/resid/wave6581_PS_.dat"
    "${CMAKE_SOURCE_DIR}/resid/wave6581_P_T.dat"
    "${CMAKE_SOURCE_DIR}/resid/wave6581__ST.dat"
    "${CMAKE_SOURCE_DIR}/resid/wave8580_PST.dat"
    "${CMAKE_SOURCE_DIR}/resid/wave8580_PS_.dat"
    "${CMAKE_SOURCE_DIR}/resid/wave8580_P_T.dat"
    "${CMAKE_SOURCE_DIR}/resid/wave8580__ST.dat"
)

set(GENERATED_WAVE_HEADERS "")

foreach(dat_file ${WAVE_DAT_FILES})
  get_filename_component(basename ${dat_file} NAME_WE)
  set(header_file "${CMAKE_SOURCE_DIR}/resid/${basename}.h")
  
  add_custom_command(
    OUTPUT ${header_file}
    COMMAND ${PERL_EXECUTABLE} ${CMAKE_SOURCE_DIR}/resid/samp2src.pl ${basename} ${dat_file} ${header_file}
    DEPENDS ${dat_file} ${CMAKE_SOURCE_DIR}/resid/samp2src.pl
    COMMENT "Generating ${header_file} from ${dat_file}"
    VERBATIM
  )
  
  list(APPEND GENERATED_WAVE_HEADERS ${header_file})
endforeach()

add_custom_target(generate_wave_headers ALL
  DEPENDS ${GENERATED_WAVE_HEADERS}
)
####################################################################################################
# End waveform header conversion integration
####################################################################################################

####################################################################################################
# External reSID Library
set(RESID_DIR "/Users/hayakzan/Desktop/SIDOsc/resid" CACHE PATH "Path to prebuilt reSID library")
include_directories("${RESID_DIR}")  # Use resid directly since headers are there
set(RESID_LIB "${RESID_DIR}/libresid.a")

if (NOT EXISTS "${RESID_LIB}")
    message(FATAL_ERROR "Could not find reSID library at ${RESID_LIB}")
endif()
####################################################################################################

####################################################################################################
# Begin target SIDOsc

set(SIDOsc_cpp_files
    plugins/SIDOsc/SIDOsc.hpp
    plugins/SIDOsc/SIDOsc.cpp
)
set(SIDOsc_sc_files
    plugins/SIDOsc/SIDOsc.sc
)
set(SIDOsc_schelp_files
    plugins/SIDOsc/SIDOsc.schelp
)

sc_add_server_plugin(
    "SIDOsc/SIDOsc" # Destination directory
    "SIDOsc"               # Target name
    "${SIDOsc_cpp_files}"
    "${SIDOsc_sc_files}"
    "${SIDOsc_schelp_files}"
)

# Link against prebuilt reSID library
target_link_libraries(SIDOsc_scsynth PRIVATE "${RESID_LIB}")
target_link_libraries(SIDOsc_supernova PRIVATE "${RESID_LIB}")

target_compile_definitions(SIDOsc_scsynth PRIVATE VERSION="1.0")
target_compile_definitions(SIDOsc_supernova PRIVATE VERSION="1.0")

# End target SIDOsc
####################################################################################################
# End plugin target definition
####################################################################################################

message(STATUS "Generating plugin targets done")
