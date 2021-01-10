cmake_minimum_required(VERSION 3.0.2)
project(sob_layer CXX)

# on gcc use -fopt-info-loop-optimized to check the auto-vectorization
# on clang use -Rpass=loop-vectorize to check the auto-vectorization
# use add_definitions("-O3 -march=native") to enable the auto-vectorization.
set(catkin_PACKAGES costmap_2d dynamic_reconfigure pluginlib)

find_package(catkin REQUIRED ${catkin_PACKAGES})
find_package(benchmark)

catkin_package(
 INCLUDE_DIRS src
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS ${catkin_PACKAGES}
)

add_library(${PROJECT_NAME} src/sob_layer.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS} src)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY src/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES sob_layer.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_test test/sob_layer.cpp)
  if(TARGET ${PROJECT_NAME}_test)
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  endif()
endif()

###############
## Benchmark ##
###############

# add --cmake-args -Dsob_layer_BENCHMARK=ON when compiling to activate this option.
set(sob_layer_BENCHMARK OFF CACHE BOOL "build the benchmark target")
if(${sob_layer_BENCHMARK})
  if(benchmark_FOUND)
    add_executable(${PROJECT_NAME}_perf perf/sob_layer.cpp)
    target_link_libraries(${PROJECT_NAME}_perf benchmark::benchmark ${PROJECT_NAME})
  else()
    message("benchmark library not found")
  endif()
endif()
