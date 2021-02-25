cmake_minimum_required(VERSION 3.0.2)
project(test_control_toolbox_mpc)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror)

find_package(ct_optcon REQUIRED)
find_package(blasfeo REQUIRED)
find_package(hpipm REQUIRED)

find_package(
    catkin REQUIRED
    COMPONENTS
    roscpp
    roslint
    dynamic_reconfigure
    message_generation
    std_msgs
)

generate_dynamic_reconfigure_options(
    cfg/test_mpc.cfg
)

add_message_files(
    FILES
    TestMPC.msg
    TestWIPSystem.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    INCLUDE_DIRS
    include
    LIBRARIES
    test_control_toolbox_mpc
    catkin_DEPENDS
    roscpp
    roslint
    dynamic_reconfigure
    message_runtime
    std_msgs
    ct_optcon
    blasfeo
    hpipm
    DEPENDS
)

include_directories(
    ./include
    ${HPIPM_INCLUDE_DIR}
    ${BLASFEO_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

#add_executable(test_control_toolbox src/test_control_toolbox.cpp)
#target_link_libraries(test_control_toolbox PUBLIC ct_optcon ${BLASFEO_LIBRARIES} ${HPIPM_LIBRARIES})
#
#add_executable(test_mpc src/test_mpc.cpp)
#add_dependencies(test_mpc ${PROJECT_NAME}_gencfg)
#target_link_libraries(test_mpc PUBLIC ct_optcon ${BLASFEO_LIBRARIES} ${HPIPM_LIBRARIES} ${catkin_LIBRARIES})
#
#add_executable(test_LTISystem src/test_LTISystem.cpp)
#target_link_libraries(test_LTISystem PUBLIC ct_optcon ${BLASFEO_LIBRARIES} ${HPIPM_LIBRARIES} ${catkin_LIBRARIES})

add_executable(test_WIPSystem src/test_WIPSystem.cpp)
target_link_libraries(test_WIPSystem PUBLIC ct_optcon ${BLASFEO_LIBRARIES} ${HPIPM_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(test_load_setting_in_file src/test_load_setting_in_file.cpp)
#target_link_libraries(test_load_setting_in_file PUBLIC ct_optcon ${BLASFEO_LIBRARIES} ${HPIPM_LIBRARIES} ${catkin_LIBRARIES})

roslint_cpp()
