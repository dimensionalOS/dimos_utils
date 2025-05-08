#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>
#include <pybind11/functional.h>
#include "tf_lcm/buffer.hpp"
#include "tf_lcm/broadcaster.hpp"
#include "tf_lcm/listener.hpp"
#include "tf_lcm/exceptions.hpp"

namespace py = pybind11;

PYBIND11_MODULE(tf_lcm_py, m) {
    m.doc() = "Python bindings for tf_lcm library";

    // Bind exceptions
    py::register_exception<tf_lcm::TransformException>(m, "TransformException");
    py::register_exception<tf_lcm::LookupException>(m, "LookupException", PyExc_RuntimeError);
    py::register_exception<tf_lcm::ConnectivityException>(m, "ConnectivityException", PyExc_RuntimeError);
    py::register_exception<tf_lcm::InvalidArgumentException>(m, "InvalidArgumentException", PyExc_ValueError);
    py::register_exception<tf_lcm::TimeoutException>(m, "TimeoutException", PyExc_RuntimeError);
    py::register_exception<tf_lcm::ExtrapolationException>(m, "ExtrapolationException", PyExc_RuntimeError);

    // Bind Buffer class
    py::class_<tf_lcm::Buffer>(m, "Buffer")
        .def(py::init<double>(), py::arg("cache_time") = 10.0)
        .def("set_transform", &tf_lcm::Buffer::setTransform, 
            py::arg("transform"), py::arg("authority"), py::arg("is_static") = false)
        .def("set_transforms", &tf_lcm::Buffer::setTransforms, 
            py::arg("transforms"), py::arg("authority"), py::arg("is_static") = false)
        .def("lookup_transform", 
            static_cast<geometry_msgs::TransformStamped (tf_lcm::Buffer::*)(
                const std::string&, 
                const std::string&, 
                const std::chrono::system_clock::time_point&,
                const std::chrono::duration<double>&)>(&tf_lcm::Buffer::lookupTransform),
            py::arg("target_frame"), py::arg("source_frame"), py::arg("time"),
            py::arg("timeout") = std::chrono::duration<double>(0.0))
        .def("lookup_transform_with_fixed_frame", 
            static_cast<geometry_msgs::TransformStamped (tf_lcm::Buffer::*)(
                const std::string&, 
                const std::chrono::system_clock::time_point&,
                const std::string&,
                const std::chrono::system_clock::time_point&,
                const std::string&,
                const std::chrono::duration<double>&)>(&tf_lcm::Buffer::lookupTransform),
            py::arg("target_frame"), py::arg("target_time"), py::arg("source_frame"),
            py::arg("source_time"), py::arg("fixed_frame"), py::arg("timeout") = std::chrono::duration<double>(0.0))
        .def("can_transform", 
            static_cast<bool (tf_lcm::Buffer::*)(
                const std::string&, 
                const std::string&, 
                const std::chrono::system_clock::time_point&,
                std::string*)>(&tf_lcm::Buffer::canTransform),
            py::arg("target_frame"), py::arg("source_frame"), py::arg("time"), py::arg("error_msg") = nullptr)
        .def("can_transform_with_fixed_frame", 
            static_cast<bool (tf_lcm::Buffer::*)(
                const std::string&, 
                const std::chrono::system_clock::time_point&,
                const std::string&,
                const std::chrono::system_clock::time_point&,
                const std::string&,
                std::string*)>(&tf_lcm::Buffer::canTransform),
            py::arg("target_frame"), py::arg("target_time"), py::arg("source_frame"),
            py::arg("source_time"), py::arg("fixed_frame"), py::arg("error_msg") = nullptr)
        .def("get_all_frame_names", &tf_lcm::Buffer::getAllFrameNames)
        .def("clear", &tf_lcm::Buffer::clear);

    // Bind TransformBroadcaster class
    py::class_<tf_lcm::TransformBroadcaster>(m, "TransformBroadcaster")
        .def(py::init<>())
        .def("send_transform", 
            static_cast<void (tf_lcm::TransformBroadcaster::*)(const geometry_msgs::TransformStamped&)>(&tf_lcm::TransformBroadcaster::sendTransform),
            py::arg("transform"))
        .def("send_transforms", 
            static_cast<void (tf_lcm::TransformBroadcaster::*)(const std::vector<geometry_msgs::TransformStamped>&)>(&tf_lcm::TransformBroadcaster::sendTransform),
            py::arg("transforms"));

    // Bind StaticTransformBroadcaster class
    py::class_<tf_lcm::StaticTransformBroadcaster>(m, "StaticTransformBroadcaster")
        .def(py::init<>())
        .def("send_transform", 
            static_cast<void (tf_lcm::StaticTransformBroadcaster::*)(const geometry_msgs::TransformStamped&)>(&tf_lcm::StaticTransformBroadcaster::sendTransform),
            py::arg("transform"))
        .def("send_transforms", 
            static_cast<void (tf_lcm::StaticTransformBroadcaster::*)(const std::vector<geometry_msgs::TransformStamped>&)>(&tf_lcm::StaticTransformBroadcaster::sendTransform),
            py::arg("transforms"));

    // Bind TransformListener class
    py::class_<tf_lcm::TransformListener>(m, "TransformListener")
        .def(py::init<double>(), py::arg("buffer_size") = 10.0)
        .def(py::init<tf_lcm::Buffer&>(), py::arg("buffer"))
        .def("get_buffer", static_cast<tf_lcm::Buffer& (tf_lcm::TransformListener::*)()>(&tf_lcm::TransformListener::getBuffer), 
             py::return_value_policy::reference_internal);
}
