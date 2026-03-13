cmake_minimum_required(VERSION 3.5)
project(g1_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Install all URDF and XML files
install(DIRECTORY .
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.urdf"
  PATTERN "*.xml"
  PATTERN "CMakeLists.txt" EXCLUDE
  PATTERN ".git" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

# Install meshes folder if it exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/meshes")
  install(DIRECTORY meshes
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

# Install RViz folder if it exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rviz")
  install(DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

ament_package()
