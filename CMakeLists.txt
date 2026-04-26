cmake_minimum_required(VERSION 3.20)

project(offline_lc_minimal VERSION 1.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(OFFLINE_LC_MINIMAL_BUILD_RUNNER "Build the offline LC runner executable." ON)
option(OFFLINE_LC_MINIMAL_BUILD_TESTS "Build lightweight native tests." ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED)
find_package(GeographicLib QUIET)
find_package(GTSAM REQUIRED)

set(OFFLINE_LC_MINIMAL_GEOGRAPHICLIB_TARGET "${GeographicLib_LIBRARIES}")
if(TARGET GeographicLib::GeographicLib)
  set(OFFLINE_LC_MINIMAL_GEOGRAPHICLIB_TARGET GeographicLib::GeographicLib)
endif()

set(OFFLINE_LC_MINIMAL_GEOGRAPHICLIB_INCLUDE_DIR "")
if(NOT GeographicLib_FOUND AND NOT TARGET GeographicLib::GeographicLib)
  find_path(
    OFFLINE_LC_MINIMAL_GEOGRAPHICLIB_INCLUDE_DIR
    NAMES GeographicLib/LocalCartesian.hpp
  )
  find_library(
    OFFLINE_LC_MINIMAL_GEOGRAPHICLIB_LIBRARY
    NAMES GeographicLib libGeographicLib.so
    PATHS /usr/lib /usr/lib/x86_64-linux-gnu /usr/local/lib /usr/local/lib64
    PATH_SUFFIXES x86_64-linux-gnu
  )
  if(NOT OFFLINE_LC_MINIMAL_GEOGRAPHICLIB_INCLUDE_DIR OR NOT OFFLINE_LC_MINIMAL_GEOGRAPHICLIB_LIBRARY)
    if(EXISTS "/usr/include/GeographicLib/LocalCartesian.hpp" AND
       EXISTS "/usr/lib/x86_64-linux-gnu/libGeographicLib.so")
      set(OFFLINE_LC_MINIMAL_GEOGRAPHICLIB_INCLUDE_DIR "/usr/include")
      set(OFFLINE_LC_MINIMAL_GEOGRAPHICLIB_LIBRARY "/usr/lib/x86_64-linux-gnu/libGeographicLib.so")
    else()
      message(FATAL_ERROR "GeographicLib headers or library were not found.")
    endif()
  endif()
  set(OFFLINE_LC_MINIMAL_GEOGRAPHICLIB_TARGET "${OFFLINE_LC_MINIMAL_GEOGRAPHICLIB_LIBRARY}")
endif()

add_library(offline_lc_minimal_lib
  src/common/Config.cpp
  src/common/GeoUtils.cpp
  src/common/ResultWriter.cpp
  src/gp/Pose3Utils.cpp
  src/io/TextDatasetLoader.cpp
  src/core/InitialStaticConstraintBuilder.cpp
  src/core/BodyZBidirectionalJumpDetector.cpp
  src/core/ImuIntegrationUtils.cpp
  src/core/ImuRateAvpReconstructor.cpp
  src/core/StaticImuAlignment.cpp
  src/core/SequentialNhcJumpDetector.cpp
  src/core/SparseVerticalJumpPlanner.cpp
  src/core/TrajectoryInitializer.cpp
  src/core/VerticalInsideBiasAdapter.cpp
  src/core/OfflineBatchRunner.cpp
)

target_include_directories(offline_lc_minimal_lib
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${OFFLINE_LC_MINIMAL_GEOGRAPHICLIB_INCLUDE_DIR}
)

target_link_libraries(offline_lc_minimal_lib
  PUBLIC
    Eigen3::Eigen
    gtsam
    ${OFFLINE_LC_MINIMAL_GEOGRAPHICLIB_TARGET}
)

set(OFFLINE_LC_MINIMAL_RUNTIME_RPATH "")
if(DEFINED GTSAM_DIR)
  get_filename_component(
    OFFLINE_LC_MINIMAL_GTSAM_LIB_DIR
    "${GTSAM_DIR}/../.."
    ABSOLUTE
  )
  list(APPEND OFFLINE_LC_MINIMAL_RUNTIME_RPATH "${OFFLINE_LC_MINIMAL_GTSAM_LIB_DIR}")
endif()
string(JOIN ":" OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH ${OFFLINE_LC_MINIMAL_RUNTIME_RPATH})

if(OFFLINE_LC_MINIMAL_BUILD_RUNNER)
  add_executable(offline_lc_runner
    src/offline_runner/main.cpp
  )

  target_link_libraries(offline_lc_runner
    PRIVATE
      offline_lc_minimal_lib
  )

  if(OFFLINE_LC_MINIMAL_RUNTIME_RPATH)
    set_target_properties(
      offline_lc_runner
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
  endif()
endif()

if(OFFLINE_LC_MINIMAL_BUILD_TESTS)
  enable_testing()

  add_executable(reweighted_combined_imu_factor_test
    tests/reweighted_combined_imu_factor_test.cpp
  )

  add_executable(vertical_rtk_preintegration_feedback_test
    tests/vertical_rtk_preintegration_feedback_test.cpp
  )

  target_link_libraries(reweighted_combined_imu_factor_test
    PRIVATE
      offline_lc_minimal_lib
  )

  target_link_libraries(vertical_rtk_preintegration_feedback_test
    PRIVATE
      offline_lc_minimal_lib
  )

  if(OFFLINE_LC_MINIMAL_RUNTIME_RPATH)
    set_target_properties(
      reweighted_combined_imu_factor_test
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
    set_target_properties(
      vertical_rtk_preintegration_feedback_test
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
  endif()

  if(OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH)
    add_test(
      NAME reweighted_combined_imu_factor_test
      COMMAND
        ${CMAKE_COMMAND} -E env
        "LD_LIBRARY_PATH=${OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
        $<TARGET_FILE:reweighted_combined_imu_factor_test>
    )
    add_test(
      NAME vertical_rtk_preintegration_feedback_test
      COMMAND
        ${CMAKE_COMMAND} -E env
        "LD_LIBRARY_PATH=${OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
        $<TARGET_FILE:vertical_rtk_preintegration_feedback_test>
    )
  else()
    add_test(
      NAME reweighted_combined_imu_factor_test
      COMMAND $<TARGET_FILE:reweighted_combined_imu_factor_test>
    )
    add_test(
      NAME vertical_rtk_preintegration_feedback_test
      COMMAND $<TARGET_FILE:vertical_rtk_preintegration_feedback_test>
    )
  endif()
endif()
