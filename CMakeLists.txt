cmake_minimum_required(VERSION 3.8)
project(coresense_vampire)

add_definitions(-w)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-g)

SET(COMPILE_TESTS OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(rosidl_default_generators REQUIRED)
find_package(coresense_msgs REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/src
	${CMAKE_SOURCE_DIR}/include/vampire
	)

link_directories(${CMAKE_SOURCE_DIR}/src)


AUX_SOURCE_DIRECTORY(${CMAKE_SOURCE_DIR}/src/Lib SOURCES_LIB)

#rosidl_generate_interfaces(${PROJECT_NAME} "action/QueryReasoner.action")

add_executable(vampire_node
  src/vampire_node.cpp
)

target_link_libraries(vampire_node
	${CMAKE_SOURCE_DIR}/lib/libvampire.so
	${CMAKE_SOURCE_DIR}/lib/libz3.so
	)

# target_include_directories(vampire_action PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
#target_compile_definitions(vampire_action
#  PRIVATE "VAMPIRE_NODE_CPP_BUILDING_DLL")

ament_target_dependencies(vampire_node
  "coresense_msgs"
  "std_msgs"
  "rclcpp"
  "rclcpp_action")

#rclcpp_components_register_node(vampire_action PLUGIN "vampire_node_cpp::VampireNode" EXECUTABLE vampire_action_server)


install(TARGETS
  vampire_node
  DESTINATION lib/${PROJECT_NAME}/)
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin)

# add_executable(vampire_node
#   src/vampire_node.cpp
#   ${VAMPIRE_DEBUG_SOURCES}
#     ${VAMPIRE_LIB_SOURCES}
#     ${VAMPIRE_LIB_SYS_SOURCES}
#     ${VAMPIRE_KERNEL_SOURCES}
#     ${VAMPIRE_INDEXING_SOURCES}
#     ${VAMPIRE_INFERENCE_SOURCES}
#     ${VAMPIRE_INSTANCEGENERATION_SOURCES}
#     ${VAMPIRE_SAT_SOURCES}
#     ${VAMPIRE_DECISION_PROCEDURES_SOURCES}
#     ${VAMPIRE_SATURATION_SOURCES}
#     ${VAMPIRE_SHELL_SOURCES}
#     ${VAMPIRE_PARSE_SOURCES}
#     ${VAMPIRE_FINITEMODELBUILDING_SOURCES}
#     ${VAMPIRE_SMTCOMP_SOURCES}
#     ${VAMPIRE_MINISAT_SOURCES}
#     ${VAMPIRE_CASC_SOURCES}
#     src/Forwards.hpp
#     "${CMAKE_CURRENT_BINARY_DIR}/version.cpp")
#     #     ${SOURCES_KERNEL}
#     #${SOURCES_LIB}
#     #${SOURCES_FMB}
#     #${SOURCES_SAT}
#     #${SOURCES_SATURATION}
#     #${SOURCES_PARSE}
#     #${SOURCES_INFERENCES}
#     #${SOURCES_LIB_SYS}
#     #   ${SOURCES_INDEXING}
#     #${SOURCES_SHELL})
#  #target_link_directories(vampire_node
#  # PUBLIC
#  # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/Lib>
#  #   $<INSTALL_INTERFACE:include>)
# ament_target_dependencies(vampire_node rclcpp std_msgs)
set(CMAKE_BUILD_TYPE )
# install(TARGETS
#   vampire_node
#   vampire_action
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
