cmake_minimum_required(VERSION 3.8)
project(uwrt_gz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim7 REQUIRED)
find_package(PythonLibs REQUIRED)

set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

# Add sources for each plugin to be registered.
add_library(MoveEntity SHARED src/MoveEntity.cc)

set_property(TARGET MoveEntity PROPERTY CXX_STANDARD 17)

#link the python-c API
target_include_directories(MoveEntity PRIVATE ${PYTHON_INCLUDE_DIRS})

target_link_libraries(MoveEntity
  PRIVATE gz-sim7::gz-sim7
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE ${PYTHON_Libraries}
)

install(
  TARGETS MoveEntity
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib 
  LIBRARY DESTINATION lib 
  RUNTIME DESTINATION bin
)


ament_package()
