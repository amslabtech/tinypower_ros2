add_library(tinypower_ros2_component SHARED
    tinypower_ros2_component.cpp
)

target_compile_definitions(tinypower_ros2_component PRIVATE "TINYPOWER_ROS2_BUILDING_DLL")

target_link_libraries(tinypower_ros2_component
)

ament_target_dependencies(tinypower_ros2_component
    rclcpp
    rclcpp_components
    geometry_msgs
    nav_msgs
    tf2_ros
    tf2_geometry_msgs
)

add_executable(tinypower_ros2_node
    tinypower_ros2_node.cpp
)

target_link_libraries(tinypower_ros2_node
    tinypower_ros2_component
)

ament_target_dependencies(tinypower_ros2_node
    rclcpp
    rclcpp_components
    geometry_msgs
    nav_msgs
)

install(TARGETS
    tinypower_ros2_node
    DESTINATION lib/tinypower_ros2
)

install(TARGETS
    tinypower_ros2_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_export_libraries(tinypower_ros2_component)
