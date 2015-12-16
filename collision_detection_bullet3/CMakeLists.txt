set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules")

set(MOVEIT_LIB_NAME moveit_collision_detection_bullet3)

set(BULLET3_ROOT "/home/mirsking/projects/gpu/bullet3/")
find_package(Bullet3)

include_directories(${BULLET3_INCLUDE_DIRS})

add_library(${MOVEIT_LIB_NAME}
  src/collision_common.cpp
  src/collision_robot_bullet3.cpp
  src/collision_world_bullet3.cpp

  src/CollisionDetectionManager/b3GpuCollisionDetectionManager.cpp
)

target_link_libraries(${MOVEIT_LIB_NAME}
        moveit_collision_detection
        ${catkin_LIBRARIES}
        ${console_bridge_LIBRARIES}
        ${urdfdom_LIBRARIES}
        ${urdfdom_headers_LIBRARIES}
        ${LIBFCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${BULLET3_LIBRARIES}
        ${CMAKE_DL_LIBS}
)
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})


add_executable(collision_detection_manager_test
  src/CollisionDetectionManager/main.cpp
  src/CollisionDetectionManager/b3GpuCollisionDetectionManager.cpp
)
target_link_libraries( collision_detection_manager_test
        ${BULLET3_LIBRARIES}
        ${CMAKE_DL_LIBS}
)

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
install(DIRECTORY include/
  DESTINATION include)
  
#if(CATKIN_ENABLE_TESTING)
#  catkin_add_gtest(test_fcl_collision_detection test/test_fcl_collision_detection.cpp)
#  target_link_libraries(test_fcl_collision_detection  ${MOVEIT_LIB_NAME} ${Boost_LIBRARIES})
#endif()
