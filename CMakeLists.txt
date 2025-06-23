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

# mc_rtc
add_project_dependency(mc_rtc REQUIRED)
if(NOT TARGET mc_rtc::mc_rtc_ros)
  message(FATAL_ERROR "mc_rtc ROS plugin is required for this controller")
endif()

find_package(baseline_walking_controller REQUIRED)

set(DOXYGEN_HTML_OUTPUT doxygen-html)
set(CATKIN_ENABLE_TESTING OFF)
option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)
set(CONFIG_OUT "${CMAKE_CURRENT_BINARY_DIR}/etc/HumanRetargetingController.yaml")
set(STATES_INSTALL_PREFIX ${MC_CONTROLLER_INSTALL_PREFIX})

configure_file(etc/HumanRetargetingController.in.yaml "${CONFIG_OUT}" @ONLY)
install(FILES ${CONFIG_OUT} DESTINATION ${MC_RTC_LIBDIR}/mc_controller/etc/)

add_subdirectory(src)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/HumanRetargetingController DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.h")

if(INSTALL_DOCUMENTATION)
  add_subdirectory(doc)
endif()
