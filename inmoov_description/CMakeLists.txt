cmake_minimum_required(VERSION 3.8)
project(inmoov_description)
find_package(ament_cmake REQUIRED)

# Install only dirs that exist (avoids "file INSTALL cannot find ..." errors)
foreach(dir IN ITEMS robots urdf xacro meshes materials textures config launch worlds)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
    install(DIRECTORY ${dir}/ DESTINATION share/${PROJECT_NAME}/${dir})
  endif()
endforeach()

ament_package()
