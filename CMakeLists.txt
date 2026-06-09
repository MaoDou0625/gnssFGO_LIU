cmake_minimum_required(VERSION 3.20)

project(offline_lc_minimal VERSION 2.2.0 LANGUAGES CXX)

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
  src/common/ResultOutputWriters.cpp
  src/common/ResultWriter.cpp
  src/factor/BazContinuityBreakCombinedImuFactor.cpp
  src/factor/VerticalMaskedCombinedImuFactor.cpp
  src/gp/Pose3Utils.cpp
  src/io/TextDatasetLoader.cpp
  src/io/BodyZBiasReestimateCsvReader.cpp
  src/io/TrajectoryCsvReader.cpp
  src/core/AttitudeReferenceConstraintBuilder.cpp
  src/core/InitialStaticConstraintBuilder.cpp
  src/core/BodyZBiasReestimateConstraintBuilder.cpp
  src/core/BodyZBiasReestimatePlanner.cpp
  src/core/BodyZBiasReestimateSourcePolicy.cpp
  src/core/BodyZBidirectionalJumpDetector.cpp
  src/core/BodyZHorizontalLeakageEstimator.cpp
  src/core/BodyZJumpConstraintWindowPlanner.cpp
  src/core/BodyZJumpWindowClassifier.cpp
  src/core/BodyZNHCConstraintBuilder.cpp
  src/core/BodyZNHCWeightPlanner.cpp
  src/core/BodyZWindowPipeline.cpp
  src/core/GnssFactorBuilder.cpp
  src/core/GnssPreOutageQualityOverride.cpp
  src/core/GnssVerticalReferenceSelector.cpp
  src/core/GraphTimelineBuilder.cpp
  src/core/ImuIntegrationUtils.cpp
  src/core/ImuRateAvpReconstructor.cpp
  src/core/InitialStaticBiasConstraintBuilder.cpp
  src/core/InitialDynamicStaticConstraintBuilder.cpp
  src/core/InitialDynamicStaticDetector.cpp
  src/core/InitialStaticPositionConstraintBuilder.cpp
  src/core/InitialStaticRtkHeightConstraintBuilder.cpp
  src/core/LateStaticDetector.cpp
  src/core/LateStaticVerticalConstraintBuilder.cpp
  src/core/RunDiagnosticsBuilder.cpp
  src/core/OptimizationStagePolicy.cpp
  src/core/RtkOutageInitialValueSmoother.cpp
  src/core/RtkOutageBiasContinuityPolicy.cpp
  src/core/RtkOutageBatchSegmentPlanner.cpp
  src/core/RtkOutageBoundaryAttitudeHandoff.cpp
  src/core/RtkOutageBoundaryAttitudeReference.cpp
  src/core/RtkOutageBoundaryBiasHandoff.cpp
  src/core/RtkOutageBoundaryConstraintBuilder.cpp
  src/core/RtkOutageBoundaryVerticalHandoff.cpp
  src/core/RtkOutageCausalReferenceBuilder.cpp
  src/core/RtkOutagePreOutageVerticalFenceBuilder.cpp
  src/core/RtkOutageRecoveryConstraintBuilder.cpp
  src/core/RtkOutageRecoveryReferenceBuilder.cpp
  src/core/RtkOutageSegmentedBatchRunner.cpp
  src/core/RtkOutageSegmentationPolicy.cpp
  src/core/RtkOutageSmoothingConstraintBuilder.cpp
  src/core/RtkOutageTerminalVelocityReference.cpp
  src/core/ResidualContributionAnalyzer.cpp
  src/core/RtkHeadingAlignmentEstimator.cpp
  src/core/RtkVelocityConstraintBuilder.cpp
  src/core/RtkOutageBazReestimatePlanner.cpp
  src/core/RtkOutageWindowPlanner.cpp
  src/core/RoadNoiseBiasReestimatePlanner.cpp
  src/core/RoadNoiseStateEstimator.cpp
  src/core/RoadNoiseStateReference.cpp
  src/core/RtkVerticalDriftGateWeighting.cpp
  src/core/RtkVerticalDriftReferenceEstimator.cpp
  src/core/RtkVerticalLowpassReferenceFilter.cpp
  src/core/Stage1OutageBodyYEnvelopeConstraintBuilder.cpp
  src/core/Stage1OutageLateralVelocityEnvelopeEstimator.cpp
  src/core/Stage1SourceReferencePolicy.cpp
  src/core/Stage1YawBranchResolver.cpp
  src/core/Stage1YawRefinementRunner.cpp
  src/core/StageAttitudeDebug.cpp
  src/core/StageAttitudeReference.cpp
  src/core/SharedVerticalReferenceBuilder.cpp
  src/core/Stage2AttitudeHoldBuilder.cpp
  src/core/Stage2HorizontalHoldBuilder.cpp
  src/core/Stage2LowfreqVerticalReferenceOptimizationRunner.cpp
  src/core/Stage2VehicleNHCConstraintBuilder.cpp
  src/core/Stage2VelocityOptimizationRunner.cpp
  src/core/Stage2VelocityReference.cpp
  src/core/Stage3HeightOptimizationPolicy.cpp
  src/core/Stage3SharedReferenceDeltaSmoother.cpp
  src/core/Stage3SharedReferenceMapper.cpp
  src/core/Stage3JumpContextEnvelopePlanner.cpp
  src/core/Stage3JumpRegularizerConstraintBuilder.cpp
  src/core/Stage3Stage2IncrementHoldConstraintBuilder.cpp
  src/core/Stage3VerticalReferenceConstraintBuilder.cpp
  src/core/Stage3VerticalReferenceOptimizationRunner.cpp
  src/core/Stage3VerticalReferenceProfilePlanner.cpp
  src/core/Stage3VerticalReferenceSmoother.cpp
  src/core/Stage3VerticalReferenceTimelineAligner.cpp
  src/core/SegmentedBatchResultAssembler.cpp
  src/core/StaticImuAlignment.cpp
  src/core/TrajectoryResultBuilder.cpp
  src/core/TrajectoryInitializer.cpp
  src/core/VerticalConstraintPolicy.cpp
  src/core/VerticalAdaptiveReweightingLoop.cpp
  src/core/VerticalAccelBiasGmConstraintBuilder.cpp
  src/core/VerticalJumpBiasConstraintBuilder.cpp
  src/core/VerticalJumpBiasSegmenter.cpp
  src/core/VerticalJumpSpectralResponseEstimator.cpp
  src/core/VerticalJumpImpulseConstraintBuilder.cpp
  src/core/VerticalJumpImuMasker.cpp
  src/core/VerticalJumpShapeConstraintBuilder.cpp
  src/core/VerticalMotionStabilityEstimator.cpp
  src/core/VerticalMotionStabilityProfile.cpp
  src/core/VerticalMotionConstraintBuilder.cpp
  src/core/VerticalPositionVelocityConsistencyConstraintBuilder.cpp
  src/core/VerticalVelocityDeltaContextScalePlanner.cpp
  src/core/VerticalVelocityDeltaSigmaModel.cpp
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

  add_executable(offline_lc_shared_vertical_reference_builder
    src/shared_vertical_reference_builder/main.cpp
  )

  add_executable(offline_lc_stage3_runner
    src/stage3_runner/main.cpp
  )

  target_link_libraries(offline_lc_runner
    PRIVATE
      offline_lc_minimal_lib
  )

  target_link_libraries(offline_lc_shared_vertical_reference_builder
    PRIVATE
      offline_lc_minimal_lib
  )

  target_link_libraries(offline_lc_stage3_runner
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
    set_target_properties(
      offline_lc_shared_vertical_reference_builder
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
    set_target_properties(
      offline_lc_stage3_runner
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
  endif()
endif()

if(OFFLINE_LC_MINIMAL_BUILD_TESTS)
  enable_testing()

  add_executable(vertical_rtk_factors_test
    tests/vertical_rtk_factors_test.cpp
  )

  add_executable(gnss_factor_builder_test
    tests/gnss_factor_builder_test.cpp
  )

  add_executable(gnss_preoutage_quality_override_test
    tests/gnss_preoutage_quality_override_test.cpp
  )

  add_executable(body_z_bidirectional_jump_detector_test
    tests/body_z_bidirectional_jump_detector_test.cpp
  )

  add_executable(config_test
    tests/config_test.cpp
  )

  add_executable(trajectory_initializer_test
    tests/trajectory_initializer_test.cpp
  )

  add_executable(trajectory_csv_reader_test
    tests/trajectory_csv_reader_test.cpp
  )

  add_executable(body_z_bias_reestimate_csv_reader_test
    tests/body_z_bias_reestimate_csv_reader_test.cpp
  )

  add_executable(attitude_reference_constraint_test
    tests/attitude_reference_constraint_test.cpp
  )

  add_executable(vertical_velocity_delta_constraint_test
    tests/vertical_velocity_delta_constraint_test.cpp
  )

  add_executable(body_z_nhc_constraint_test
    tests/body_z_nhc_constraint_test.cpp
  )

  add_executable(vertical_jump_constraints_test
    tests/vertical_jump_constraints_test.cpp
  )

  add_executable(road_noise_state_estimator_test
    tests/road_noise_state_estimator_test.cpp
  )

  add_executable(vertical_position_velocity_consistency_constraint_test
    tests/vertical_position_velocity_consistency_constraint_test.cpp
  )

  add_executable(static_vertical_bias_constraint_test
    tests/static_vertical_bias_constraint_test.cpp
  )

  add_executable(rtk_vertical_drift_reference_test
    tests/rtk_vertical_drift_reference_test.cpp
  )

  add_executable(late_static_detector_test
    tests/late_static_detector_test.cpp
  )

  add_executable(rtk_velocity_constraint_test
    tests/rtk_velocity_constraint_test.cpp
  )

  add_executable(rtk_outage_window_planner_test
    tests/rtk_outage_window_planner_test.cpp
  )

  add_executable(rtk_outage_recovery_constraint_test
    tests/rtk_outage_recovery_constraint_test.cpp
  )

  add_executable(residual_contribution_analyzer_test
    tests/residual_contribution_analyzer_test.cpp
  )

  add_executable(stage1_yaw_refinement_test
    tests/stage1_yaw_refinement_test.cpp
  )

  add_executable(stage_attitude_reference_test
    tests/stage_attitude_reference_test.cpp
  )

  add_executable(shared_vertical_reference_builder_test
    tests/shared_vertical_reference_builder_test.cpp
  )

  add_executable(stage2_velocity_optimization_test
    tests/stage2_velocity_optimization_test.cpp
  )

  add_executable(stage3_vertical_reference_test
    tests/stage3_vertical_reference_test.cpp
  )

  add_executable(stage3_shared_reference_mapper_test
    tests/stage3_shared_reference_mapper_test.cpp
  )

  target_compile_definitions(config_test
    PRIVATE
      OFFLINE_LC_MINIMAL_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
  )

  target_link_libraries(vertical_rtk_factors_test
    PRIVATE
      offline_lc_minimal_lib
  )

  target_link_libraries(gnss_factor_builder_test
    PRIVATE
      offline_lc_minimal_lib
  )

  target_link_libraries(gnss_preoutage_quality_override_test
    PRIVATE
      offline_lc_minimal_lib
  )

  target_link_libraries(body_z_bidirectional_jump_detector_test
    PRIVATE
      offline_lc_minimal_lib
  )

  target_link_libraries(config_test
    PRIVATE
      offline_lc_minimal_lib
  )

  target_link_libraries(trajectory_initializer_test
    PRIVATE
      offline_lc_minimal_lib
  )

  target_link_libraries(trajectory_csv_reader_test
    PRIVATE
      offline_lc_minimal_lib
  )

  target_link_libraries(body_z_bias_reestimate_csv_reader_test
    PRIVATE
      offline_lc_minimal_lib
  )

  target_link_libraries(attitude_reference_constraint_test
    PRIVATE
      offline_lc_minimal_lib
  )

  target_link_libraries(vertical_velocity_delta_constraint_test
    PRIVATE
      offline_lc_minimal_lib
  )

  target_link_libraries(body_z_nhc_constraint_test
    PRIVATE
      offline_lc_minimal_lib
  )

  target_link_libraries(vertical_jump_constraints_test
    PRIVATE
      offline_lc_minimal_lib
  )

  target_link_libraries(road_noise_state_estimator_test
    PRIVATE
      offline_lc_minimal_lib
  )

  target_link_libraries(vertical_position_velocity_consistency_constraint_test
    PRIVATE
      offline_lc_minimal_lib
  )

  target_link_libraries(static_vertical_bias_constraint_test
    PRIVATE
      offline_lc_minimal_lib
  )

  target_link_libraries(rtk_vertical_drift_reference_test
    PRIVATE
      offline_lc_minimal_lib
  )

  target_link_libraries(late_static_detector_test
    PRIVATE
      offline_lc_minimal_lib
  )

  target_link_libraries(rtk_velocity_constraint_test
    PRIVATE
      offline_lc_minimal_lib
  )

  target_link_libraries(rtk_outage_window_planner_test
    PRIVATE
      offline_lc_minimal_lib
  )

  target_link_libraries(rtk_outage_recovery_constraint_test
    PRIVATE
      offline_lc_minimal_lib
  )

  target_link_libraries(residual_contribution_analyzer_test
    PRIVATE
      offline_lc_minimal_lib
  )

  target_link_libraries(stage1_yaw_refinement_test
    PRIVATE
      offline_lc_minimal_lib
  )

  target_link_libraries(stage_attitude_reference_test
    PRIVATE
      offline_lc_minimal_lib
  )

  target_link_libraries(shared_vertical_reference_builder_test
    PRIVATE
      offline_lc_minimal_lib
  )

  target_link_libraries(stage2_velocity_optimization_test
    PRIVATE
      offline_lc_minimal_lib
  )

  target_link_libraries(stage3_vertical_reference_test
    PRIVATE
      offline_lc_minimal_lib
  )

  target_link_libraries(stage3_shared_reference_mapper_test
    PRIVATE
      offline_lc_minimal_lib
  )

  if(OFFLINE_LC_MINIMAL_RUNTIME_RPATH)
    set_target_properties(
      vertical_rtk_factors_test
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
    set_target_properties(
      gnss_factor_builder_test
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
    set_target_properties(
      gnss_preoutage_quality_override_test
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
    set_target_properties(
      body_z_bidirectional_jump_detector_test
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
    set_target_properties(
      config_test
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
    set_target_properties(
      trajectory_initializer_test
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
    set_target_properties(
      trajectory_csv_reader_test
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
    set_target_properties(
      body_z_bias_reestimate_csv_reader_test
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
    set_target_properties(
      attitude_reference_constraint_test
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
    set_target_properties(
      vertical_velocity_delta_constraint_test
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
    set_target_properties(
      body_z_nhc_constraint_test
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
    set_target_properties(
      vertical_jump_constraints_test
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
    set_target_properties(
      road_noise_state_estimator_test
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
    set_target_properties(
      vertical_position_velocity_consistency_constraint_test
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
    set_target_properties(
      static_vertical_bias_constraint_test
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
    set_target_properties(
      rtk_vertical_drift_reference_test
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
    set_target_properties(
      late_static_detector_test
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
    set_target_properties(
      rtk_velocity_constraint_test
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
    set_target_properties(
      rtk_outage_window_planner_test
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
    set_target_properties(
      rtk_outage_recovery_constraint_test
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
    set_target_properties(
      residual_contribution_analyzer_test
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
    set_target_properties(
      stage1_yaw_refinement_test
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
    set_target_properties(
      stage_attitude_reference_test
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
    set_target_properties(
      shared_vertical_reference_builder_test
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
    set_target_properties(
      stage2_velocity_optimization_test
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
    set_target_properties(
      stage3_vertical_reference_test
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
    set_target_properties(
      stage3_shared_reference_mapper_test
      PROPERTIES
        BUILD_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
        INSTALL_RPATH "${OFFLINE_LC_MINIMAL_RUNTIME_RPATH}"
    )
  endif()

  if(OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH)
    add_test(
      NAME vertical_rtk_factors_test
      COMMAND
        ${CMAKE_COMMAND} -E env
        "LD_LIBRARY_PATH=${OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
        $<TARGET_FILE:vertical_rtk_factors_test>
    )
    add_test(
      NAME body_z_bidirectional_jump_detector_test
      COMMAND
        ${CMAKE_COMMAND} -E env
        "LD_LIBRARY_PATH=${OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
        $<TARGET_FILE:body_z_bidirectional_jump_detector_test>
    )
    add_test(
      NAME gnss_factor_builder_test
      COMMAND
        ${CMAKE_COMMAND} -E env
        "LD_LIBRARY_PATH=${OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
        $<TARGET_FILE:gnss_factor_builder_test>
    )
    add_test(
      NAME gnss_preoutage_quality_override_test
      COMMAND
        ${CMAKE_COMMAND} -E env
        "LD_LIBRARY_PATH=${OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
        $<TARGET_FILE:gnss_preoutage_quality_override_test>
    )
    add_test(
      NAME config_test
      COMMAND
        ${CMAKE_COMMAND} -E env
        "LD_LIBRARY_PATH=${OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
        $<TARGET_FILE:config_test>
    )
    add_test(
      NAME trajectory_initializer_test
      COMMAND
        ${CMAKE_COMMAND} -E env
        "LD_LIBRARY_PATH=${OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
        $<TARGET_FILE:trajectory_initializer_test>
    )
    add_test(
      NAME trajectory_csv_reader_test
      COMMAND
        ${CMAKE_COMMAND} -E env
        "LD_LIBRARY_PATH=${OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
        $<TARGET_FILE:trajectory_csv_reader_test>
    )
    add_test(
      NAME body_z_bias_reestimate_csv_reader_test
      COMMAND
        ${CMAKE_COMMAND} -E env
        "LD_LIBRARY_PATH=${OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
        $<TARGET_FILE:body_z_bias_reestimate_csv_reader_test>
    )
    add_test(
      NAME attitude_reference_constraint_test
      COMMAND
        ${CMAKE_COMMAND} -E env
        "LD_LIBRARY_PATH=${OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
        $<TARGET_FILE:attitude_reference_constraint_test>
    )
    add_test(
      NAME vertical_velocity_delta_constraint_test
      COMMAND
        ${CMAKE_COMMAND} -E env
        "LD_LIBRARY_PATH=${OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
        $<TARGET_FILE:vertical_velocity_delta_constraint_test>
    )
    add_test(
      NAME body_z_nhc_constraint_test
      COMMAND
        ${CMAKE_COMMAND} -E env
        "LD_LIBRARY_PATH=${OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
        $<TARGET_FILE:body_z_nhc_constraint_test>
    )
    add_test(
      NAME vertical_jump_constraints_test
      COMMAND
        ${CMAKE_COMMAND} -E env
        "LD_LIBRARY_PATH=${OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
        $<TARGET_FILE:vertical_jump_constraints_test>
    )
    add_test(
      NAME road_noise_state_estimator_test
      COMMAND
        ${CMAKE_COMMAND} -E env
        "LD_LIBRARY_PATH=${OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
        $<TARGET_FILE:road_noise_state_estimator_test>
    )
    add_test(
      NAME vertical_position_velocity_consistency_constraint_test
      COMMAND
        ${CMAKE_COMMAND} -E env
        "LD_LIBRARY_PATH=${OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
        $<TARGET_FILE:vertical_position_velocity_consistency_constraint_test>
    )
    add_test(
      NAME static_vertical_bias_constraint_test
      COMMAND
        ${CMAKE_COMMAND} -E env
        "LD_LIBRARY_PATH=${OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
        $<TARGET_FILE:static_vertical_bias_constraint_test>
    )
    add_test(
      NAME rtk_vertical_drift_reference_test
      COMMAND
        ${CMAKE_COMMAND} -E env
        "LD_LIBRARY_PATH=${OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
        $<TARGET_FILE:rtk_vertical_drift_reference_test>
    )
    add_test(
      NAME late_static_detector_test
      COMMAND
        ${CMAKE_COMMAND} -E env
        "LD_LIBRARY_PATH=${OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
        $<TARGET_FILE:late_static_detector_test>
    )
    add_test(
      NAME rtk_velocity_constraint_test
      COMMAND
        ${CMAKE_COMMAND} -E env
        "LD_LIBRARY_PATH=${OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
        $<TARGET_FILE:rtk_velocity_constraint_test>
    )
    add_test(
      NAME rtk_outage_window_planner_test
      COMMAND
        ${CMAKE_COMMAND} -E env
        "LD_LIBRARY_PATH=${OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
        $<TARGET_FILE:rtk_outage_window_planner_test>
    )
    add_test(
      NAME rtk_outage_recovery_constraint_test
      COMMAND
        ${CMAKE_COMMAND} -E env
        "LD_LIBRARY_PATH=${OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
        $<TARGET_FILE:rtk_outage_recovery_constraint_test>
    )
    add_test(
      NAME residual_contribution_analyzer_test
      COMMAND
        ${CMAKE_COMMAND} -E env
        "LD_LIBRARY_PATH=${OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
        $<TARGET_FILE:residual_contribution_analyzer_test>
    )
    add_test(
      NAME stage1_yaw_refinement_test
      COMMAND
        ${CMAKE_COMMAND} -E env
        "LD_LIBRARY_PATH=${OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
        $<TARGET_FILE:stage1_yaw_refinement_test>
    )
    add_test(
      NAME stage_attitude_reference_test
      COMMAND
        ${CMAKE_COMMAND} -E env
        "LD_LIBRARY_PATH=${OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
        $<TARGET_FILE:stage_attitude_reference_test>
    )
    add_test(
      NAME shared_vertical_reference_builder_test
      COMMAND
        ${CMAKE_COMMAND} -E env
        "LD_LIBRARY_PATH=${OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
        $<TARGET_FILE:shared_vertical_reference_builder_test>
    )
    add_test(
      NAME stage2_velocity_optimization_test
      COMMAND
        ${CMAKE_COMMAND} -E env
        "LD_LIBRARY_PATH=${OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
        $<TARGET_FILE:stage2_velocity_optimization_test>
    )
    add_test(
      NAME stage3_vertical_reference_test
      COMMAND
        ${CMAKE_COMMAND} -E env
        "LD_LIBRARY_PATH=${OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
        $<TARGET_FILE:stage3_vertical_reference_test>
    )
    add_test(
      NAME stage3_shared_reference_mapper_test
      COMMAND
        ${CMAKE_COMMAND} -E env
        "LD_LIBRARY_PATH=${OFFLINE_LC_MINIMAL_RUNTIME_LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH}"
        $<TARGET_FILE:stage3_shared_reference_mapper_test>
    )
  else()
    add_test(
      NAME vertical_rtk_factors_test
      COMMAND $<TARGET_FILE:vertical_rtk_factors_test>
    )
    add_test(
      NAME body_z_bidirectional_jump_detector_test
      COMMAND $<TARGET_FILE:body_z_bidirectional_jump_detector_test>
    )
    add_test(
      NAME gnss_factor_builder_test
      COMMAND $<TARGET_FILE:gnss_factor_builder_test>
    )
    add_test(
      NAME gnss_preoutage_quality_override_test
      COMMAND $<TARGET_FILE:gnss_preoutage_quality_override_test>
    )
    add_test(
      NAME config_test
      COMMAND $<TARGET_FILE:config_test>
    )
    add_test(
      NAME trajectory_initializer_test
      COMMAND $<TARGET_FILE:trajectory_initializer_test>
    )
    add_test(
      NAME trajectory_csv_reader_test
      COMMAND $<TARGET_FILE:trajectory_csv_reader_test>
    )
    add_test(
      NAME body_z_bias_reestimate_csv_reader_test
      COMMAND $<TARGET_FILE:body_z_bias_reestimate_csv_reader_test>
    )
    add_test(
      NAME attitude_reference_constraint_test
      COMMAND $<TARGET_FILE:attitude_reference_constraint_test>
    )
    add_test(
      NAME vertical_velocity_delta_constraint_test
      COMMAND $<TARGET_FILE:vertical_velocity_delta_constraint_test>
    )
    add_test(
      NAME body_z_nhc_constraint_test
      COMMAND $<TARGET_FILE:body_z_nhc_constraint_test>
    )
    add_test(
      NAME vertical_jump_constraints_test
      COMMAND $<TARGET_FILE:vertical_jump_constraints_test>
    )
    add_test(
      NAME road_noise_state_estimator_test
      COMMAND $<TARGET_FILE:road_noise_state_estimator_test>
    )
    add_test(
      NAME vertical_position_velocity_consistency_constraint_test
      COMMAND $<TARGET_FILE:vertical_position_velocity_consistency_constraint_test>
    )
    add_test(
      NAME static_vertical_bias_constraint_test
      COMMAND $<TARGET_FILE:static_vertical_bias_constraint_test>
    )
    add_test(
      NAME rtk_vertical_drift_reference_test
      COMMAND $<TARGET_FILE:rtk_vertical_drift_reference_test>
    )
    add_test(
      NAME rtk_velocity_constraint_test
      COMMAND $<TARGET_FILE:rtk_velocity_constraint_test>
    )
    add_test(
      NAME rtk_outage_window_planner_test
      COMMAND $<TARGET_FILE:rtk_outage_window_planner_test>
    )
    add_test(
      NAME rtk_outage_recovery_constraint_test
      COMMAND $<TARGET_FILE:rtk_outage_recovery_constraint_test>
    )
    add_test(
      NAME residual_contribution_analyzer_test
      COMMAND $<TARGET_FILE:residual_contribution_analyzer_test>
    )
    add_test(
      NAME stage1_yaw_refinement_test
      COMMAND $<TARGET_FILE:stage1_yaw_refinement_test>
    )
    add_test(
      NAME stage_attitude_reference_test
      COMMAND $<TARGET_FILE:stage_attitude_reference_test>
    )
    add_test(
      NAME shared_vertical_reference_builder_test
      COMMAND $<TARGET_FILE:shared_vertical_reference_builder_test>
    )
    add_test(
      NAME stage2_velocity_optimization_test
      COMMAND $<TARGET_FILE:stage2_velocity_optimization_test>
    )
    add_test(
      NAME stage3_vertical_reference_test
      COMMAND $<TARGET_FILE:stage3_vertical_reference_test>
    )
    add_test(
      NAME stage3_shared_reference_mapper_test
      COMMAND $<TARGET_FILE:stage3_shared_reference_mapper_test>
    )
  endif()
endif()
