// example_class.cpp
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "arx_l5pro_api/interfaces/InterfacesPy.hpp"

namespace py = pybind11;

PYBIND11_MODULE(arx5_interface, m) {
    py::class_<arx::L5ProInterfacesPy>(m, "InterfacesPy")
        .def(py::init<std::string>()) 
        //.def("get_joint_names", &arx::L5ProInterfacesPy::get_joint_names)
        .def("go_home", &arx::L5ProInterfacesPy::go_home)
        //.def("set_joint_positions", &arx::L5ProInterfacesPy::set_joint_positions)
        //.def("set_joint_velocities", &arx::L5ProInterfacesPy::set_joint_velocities)
        .def("set_ee_pose", &arx::L5ProInterfacesPy::set_ee_pose)
        .def("get_joint_positions", &arx::L5ProInterfacesPy::get_joint_positions)
        .def("get_joint_velocities", &arx::L5ProInterfacesPy::get_joint_velocities)
        .def("get_ee_pose", &arx::L5ProInterfacesPy::get_ee_pose);
}