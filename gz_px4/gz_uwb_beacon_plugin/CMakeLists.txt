cmake_minimum_required(VERSION 3.8.0)
project(gz_uwb_beacon_plugin)

# Establecer C++14 como estándar por defecto
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Encontrar dependencias
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gz-sim7 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-transport12 REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Generar interfaces de mensajes
rosidl_generate_interfaces(${PROJECT_NAME}_msgs
  "msg/Ranging.msg"
  DEPENDENCIES std_msgs
)

# Hacer que los mensajes generados estén disponibles en tiempo de compilación
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_msgs "rosidl_typesupport_cpp")

# Crear la biblioteca del plugin
add_library(gz_uwb_beacon_plugin SHARED src/gz_uwb_beacon_plugin.cpp)

target_include_directories(gz_uwb_beacon_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Dependencias
ament_target_dependencies(gz_uwb_beacon_plugin
  rclcpp
  gz-sim7
  gz-plugin2
  gz-common5
  gz-math7
  gz-transport12
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Vincular con los mensajes generados
target_link_libraries(gz_uwb_beacon_plugin "${cpp_typesupport_target}")

# Instalar el plugin
install(
  TARGETS gz_uwb_beacon_plugin
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Exportar dependencias
ament_export_include_directories(include)
ament_export_libraries(gz_uwb_beacon_plugin)
ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  gz-sim7
  gz-plugin2
  gz-common5
  gz-math7
  gz-transport12
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rosidl_default_runtime
)

# Añadir las interfaces generadas a las exportaciones
ament_export_dependencies(rosidl_default_runtime)

ament_package()