cmake_minimum_required(VERSION 3.8)
project(pm_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/GripperForces.msg"
  "msg/VizAdhesivePoint.msg"
  "msg/VizAdhesivePoints.msg"

  "srv/DisppenseAtPoints.srv"
  "srv/EmptyWithSuccess.srv"
  "srv/UVSetOnOff.srv"
  "srv/UVGetOnOff.srv"
  "srv/UVSetPower.srv"
  "srv/UVGetPower.srv"
  "srv/UVSetTime.srv"
  "srv/UVGetTime.srv"
  "srv/UVCuringSkill.srv"
  "srv/CoaxLightGetState.srv"
  "srv/CoaxLightSetState.srv"
  "srv/Cam2LightSetState.srv"
  "srv/Cam2LightGetState.srv"
  "srv/RingLightGetState.srv"
  "srv/RingLightSetState.srv"
  "srv/PneumaticSetPosition.srv"
  "srv/PneumaticGetPosition.srv"
  "srv/NozzleSetPosition.srv"
  "srv/NozzleGetPosition.srv"
  "srv/ForceSensorGetMeasurement.srv"
  "srv/ForceSensorBias.srv"
  "srv/LaserGetMeasurement.srv"
  "srv/ReferenceCubeState.srv"
  "srv/DispenseForTime.srv"
  "srv/CreateVizAdhesivePoint.srv"
  "srv/GripperMove.srv"
  "srv/GripperMoveRel.srv"
  "srv/GripperSetVel.srv"
  "srv/GripperSetAccel.srv"
  "srv/GripperGetVel.srv"
  "srv/GripperGetAccel.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
