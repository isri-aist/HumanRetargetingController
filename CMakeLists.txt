cmake_minimum_required(VERSION 3.10)

set(PROJECT_NAME human_retargeting_controller)
set(PROJECT_GENERATED_HEADERS_SKIP_DEPRECATED ON)
set(PROJECT_GENERATED_HEADERS_SKIP_CONFIG ON)
set(PROJECT_GENERATED_HEADERS_SKIP_WARNING ON)
set(PROJECT_URL https://github.com/isri-aist/HumanRetargetingController)
set(PROJECT_DESCRIPTION "")
set(CMAKE_CXX_STANDARD 17)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CXX_DISABLE_WERROR ON)
set(CMAKE_COLOR_DIAGNOSTICS ON)

option(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)

include(cmake/base.cmake)
project(human_retargeting_controller LANGUAGES CXX)

add_compile_options("-fdiagnostics-color")

# mc_rtc
add_project_dependency(mc_rtc REQUIRED)
if(NOT TARGET mc_rtc::mc_rtc_ros)
  message(FATAL_ERROR "mc_rtc ROS plugin is required for this controller")
endif()

if(DEFINED CATKIN_DEVEL_PREFIX)
  set(DOXYGEN_HTML_OUTPUT html)
  find_package(catkin REQUIRED COMPONENTS
    baseline_walking_controller
  )

  # Eigen
  find_package(Eigen3 REQUIRED)
  include_directories(${EIGEN3_INCLUDE_DIR})

  catkin_package(
    CATKIN_DEPENDS
    baseline_walking_controller
    DEPENDS EIGEN3
    INCLUDE_DIRS include
    LIBRARIES HumanRetargetingController
  )

  include_directories(include ${catkin_INCLUDE_DIRS})
  link_directories(${catkin_LIBRARY_DIRS})

  set(BWC_STATES_LIBRARIES "${baseline_walking_controller_DEVEL_PREFIX}/lib/baseline_walking_controller/states")
  set(BWC_STATES_FILES "${BWC_STATES_LIBRARIES}/data")

  set(CONFIG_OUT "${CATKIN_DEVEL_PREFIX}/lib/mc_controller/etc/HumanRetargetingController.yaml")
  set(CONFIG_DIR_STANDALONE "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/HumanRetargetingController")
  set(CONFIG_DIR_CATKIN "${CATKIN_DEVEL_PREFIX}/lib/mc_controller/HumanRetargetingController")
  if(EXISTS "${CONFIG_DIR_STANDALONE}" AND IS_DIRECTORY "${CONFIG_DIR_STANDALONE}")
    execute_process(COMMAND cmake -E create_symlink
      "${CONFIG_DIR_STANDALONE}" "${CONFIG_DIR_CATKIN}"
    )
  endif()
  configure_file(etc/mc_rtc.in.yaml ${PROJECT_SOURCE_DIR}/etc/mc_rtc.yaml @ONLY)
  set(STATES_INSTALL_PREFIX ${CATKIN_DEVEL_PREFIX}/lib)
else()
  set(DOXYGEN_HTML_OUTPUT doxygen-html)
  set(CATKIN_ENABLE_TESTING OFF)
  option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)
  add_project_dependency(baseline_walking_controller REQUIRED)
  set(CONFIG_OUT "${CMAKE_CURRENT_BINARY_DIR}/etc/HumanRetargetingController.yaml")
  set(STATES_INSTALL_PREFIX ${MC_CONTROLLER_INSTALL_PREFIX})
endif()

configure_file(etc/HumanRetargetingController.in.yaml "${CONFIG_OUT}" @ONLY)
install(FILES ${CONFIG_OUT} DESTINATION ${MC_RTC_LIBDIR}/mc_controller/etc/)

add_subdirectory(src)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/HumanRetargetingController DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.h")

if(INSTALL_DOCUMENTATION)
  add_subdirectory(doc)
endif()
