cmake_minimum_required(VERSION 2.8.3)
project(performance_tests)

add_compile_options(-std=c++11)

set(ENV{ROS_LANG_DISABLE} "genlisp")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  dynamic_reconfigure
  roscpp
  rospy
)

add_message_files(
    FILES
    SuperAwesome.msg
    Frequency.msg
)

generate_messages( DEPENDENCIES )

generate_dynamic_reconfigure_options(
    cfg/Publisher.cfg
)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

file(GLOB HEADERS include/performance_tests/*.h)
add_executable(publisher_cpp src/publisher_cpp.cpp ${HEADERS})
add_dependencies(publisher_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(publisher_cpp ${catkin_LIBRARIES})

add_executable(subscriber_cpp src/subscriber_cpp.cpp ${HEADERS})
add_dependencies(subscriber_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(subscriber_cpp ${catkin_LIBRARIES})
