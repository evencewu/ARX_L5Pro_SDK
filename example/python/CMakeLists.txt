cmake_minimum_required(VERSION 3.10)
project(arx_l5pro_python_api VERSION 1.0)

find_package(pybind11 REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/lib/arx_l5pro_api/include)
include_directories(${PROJECT_SOURCE_DIR}/lib/arx_hardware_interface/include)

pybind11_add_module(arx5_interface src/google_single_arm_interface.cpp)
target_link_libraries(arx5_interface PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_l5pro_api/libarx_l5pro_api.so ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_hardware_interface/libarx_hardware_interface.so pybind11::module)