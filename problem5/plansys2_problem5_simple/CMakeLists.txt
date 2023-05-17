cmake_minimum_required(VERSION 3.5)
project(plansys2_problem5_simple)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    plansys2_msgs
    plansys2_executor
)

add_executable(fill_box_node src/fill_box_node.cpp)
ament_target_dependencies(fill_box_node ${dependencies})

add_executable(load_carrier_node src/load_carrier_node.cpp)
ament_target_dependencies(load_carrier_node ${dependencies})

add_executable(move_carrier_with_box_node src/move_carrier_with_box_node.cpp)
ament_target_dependencies(move_carrier_with_box_node ${dependencies})

add_executable(unload_carrier_node src/unload_carrier_node.cpp)
ament_target_dependencies(unload_carrier_node ${dependencies})

add_executable(unfill_box_node src/unfill_box_node.cpp)
ament_target_dependencies(unfill_box_node ${dependencies})

add_executable(deliver_food_node src/deliver_food_node.cpp)
ament_target_dependencies(deliver_food_node ${dependencies})

add_executable(deliver_medicine_node src/deliver_medicine_node.cpp)
ament_target_dependencies(deliver_medicine_node ${dependencies})

add_executable(deliver_tool_node src/deliver_tool_node.cpp)
ament_target_dependencies(deliver_tool_node ${dependencies})

add_executable(move_carrier_without_box_node src/move_carrier_without_box_node.cpp)
ament_target_dependencies(move_carrier_without_box_node ${dependencies})

install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

install(TARGETS
  fill_box_node
  load_carrier_node
  move_carrier_with_box_node
  unload_carrier_node
  unfill_box_node
  deliver_food_node
  deliver_medicine_node
  deliver_tool_node
  move_carrier_without_box_node
  
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()
