cmake_minimum_required(VERSION 3.8)
project(bob_av_tools)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME}
  SETUP_CFG
    ${PROJECT_NAME}/setup.cfg
  SCRIPTS_DESTINATION
    lib/${PROJECT_NAME}
)

# Install assets (HTML and Vendor libs) to the package directory
# This allows the node to find them regardless of install location
install(FILES
  bob_av_tools/overlay.html
  bob_av_tools/index.html
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  bob_av_tools/vendor
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(
    PROGRAMS
    "scripts/write_fifo.sh"
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
