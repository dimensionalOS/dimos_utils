#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <lcm/lcm-cpp.hpp>
#include "tf_lcm/buffer.hpp"
#include "tf_lcm/broadcaster.hpp"
#include "tf_lcm/listener.hpp"
#include "tf_lcm/exceptions.hpp"

namespace py = pybind11;

// Wrapper for lcm::LCM to make it work with Python
class PyLCM : public lcm::LCM {
public:
    using lcm::LCM::LCM;  // Inherit constructors
    
    // Python-friendly handle method that can be called from Python
    void handle() {
        lcm::LCM::handle();
    }
    
    // Python-friendly handleTimeout method
    bool handleTimeout(int timeout_ms) {
        return lcm::LCM::handleTimeout(timeout_ms);
    }
};

// Helper functions to convert between Python and C++ types

// Convert from Python TransformStamped to C++ geometry_msgs::TransformStamped
geometry_msgs::TransformStamped convertPyTransformToCpp(py::object pyTransform) {
    geometry_msgs::TransformStamped transform;
    
    // Extract header information
    py::object pyHeader = pyTransform.attr("header");
    transform.header.frame_id = py::str(pyHeader.attr("frame_id"));
    transform.header.stamp.sec = py::int_(pyHeader.attr("stamp").attr("sec"));
    transform.header.stamp.nsec = py::int_(pyHeader.attr("stamp").attr("nsec"));
    
    // Extract child_frame_id
    transform.child_frame_id = py::str(pyTransform.attr("child_frame_id"));
    
    // Extract transform data
    py::object pyTf = pyTransform.attr("transform");
    py::object pyTrans = pyTf.attr("translation");
    py::object pyRot = pyTf.attr("rotation");
    
    // Translation
    transform.transform.translation.x = py::float_(pyTrans.attr("x"));
    transform.transform.translation.y = py::float_(pyTrans.attr("y"));
    transform.transform.translation.z = py::float_(pyTrans.attr("z"));
    
    // Rotation
    transform.transform.rotation.x = py::float_(pyRot.attr("x"));
    transform.transform.rotation.y = py::float_(pyRot.attr("y"));
    transform.transform.rotation.z = py::float_(pyRot.attr("z"));
    transform.transform.rotation.w = py::float_(pyRot.attr("w"));
    
    return transform;
}

// Convert from C++ geometry_msgs::TransformStamped to Python TransformStamped
py::object convertCppTransformToPy(const geometry_msgs::TransformStamped& cppTransform, py::object lcm_module) {
    // If lcm_module is None, try to import it
    if (lcm_module.is_none()) {
        py::module sys = py::module::import("sys");
        py::module os = py::module::import("os");
        
        // Add the parent directory to sys.path
        py::str dirname = os.attr("path").attr("dirname")(os.attr("path").attr("dirname")(os.attr("path").attr("abspath")("__file__")));
        sys.attr("path").attr("append")(dirname);
    }
    
    // Import the specific message types directly using their correct import paths
    py::object TransformStamped;
    py::object Transform;
    py::object Vector3;
    py::object Quaternion;
    py::object Header;
    
    try {
        // Import the required message types directly from their modules
        TransformStamped = py::module::import("lcm_msgs.geometry_msgs.TransformStamped").attr("TransformStamped");
        Transform = py::module::import("lcm_msgs.geometry_msgs.Transform").attr("Transform");
        Vector3 = py::module::import("lcm_msgs.geometry_msgs.Vector3").attr("Vector3");
        Quaternion = py::module::import("lcm_msgs.geometry_msgs.Quaternion").attr("Quaternion");
        Header = py::module::import("lcm_msgs.std_msgs.Header").attr("Header");
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("Error importing LCM message types: ") + e.what());
    }
    
    // Create Python objects
    py::object pyTransform = TransformStamped();
    py::object pyHeader = Header();
    py::object pyTf = Transform();
    py::object pyTrans = Vector3();
    py::object pyRot = Quaternion();
    
    // Set header information
    pyHeader.attr("frame_id") = py::str(cppTransform.header.frame_id);
    py::object stamp = pyHeader.attr("stamp");
    stamp.attr("sec") = py::int_(cppTransform.header.stamp.sec);
    stamp.attr("nsec") = py::int_(cppTransform.header.stamp.nsec);
    
    // Set child_frame_id
    pyTransform.attr("child_frame_id") = py::str(cppTransform.child_frame_id);
    
    // Set translation
    pyTrans.attr("x") = py::float_(cppTransform.transform.translation.x);
    pyTrans.attr("y") = py::float_(cppTransform.transform.translation.y);
    pyTrans.attr("z") = py::float_(cppTransform.transform.translation.z);
    
    // Set rotation
    pyRot.attr("x") = py::float_(cppTransform.transform.rotation.x);
    pyRot.attr("y") = py::float_(cppTransform.transform.rotation.y);
    pyRot.attr("z") = py::float_(cppTransform.transform.rotation.z);
    pyRot.attr("w") = py::float_(cppTransform.transform.rotation.w);
    
    // Assemble the transform
    pyTf.attr("translation") = pyTrans;
    pyTf.attr("rotation") = pyRot;
    pyTransform.attr("header") = pyHeader;
    pyTransform.attr("transform") = pyTf;
    
    return pyTransform;
}

PYBIND11_MODULE(_tf_lcm_py, m) {
    m.doc() = "Python bindings for tf_lcm library";

    // Bind exceptions
    py::register_exception<tf_lcm::TransformException>(m, "TransformException");
    py::register_exception<tf_lcm::LookupException>(m, "LookupException", PyExc_RuntimeError);
    py::register_exception<tf_lcm::ConnectivityException>(m, "ConnectivityException", PyExc_RuntimeError);
    py::register_exception<tf_lcm::InvalidArgumentException>(m, "InvalidArgumentException", PyExc_ValueError);
    py::register_exception<tf_lcm::TimeoutException>(m, "TimeoutException", PyExc_RuntimeError);
    py::register_exception<tf_lcm::ExtrapolationException>(m, "ExtrapolationException", PyExc_RuntimeError);

    // Helper function to convert Python datetime to C++ time_point
    m.def("py_datetime_to_timepoint", [](py::object datetime_obj) {
        // Extract timestamp from Python datetime
        auto timestamp = py::cast<double>(datetime_obj.attr("timestamp")());
        // Convert seconds since epoch to system_clock time_point
        return std::chrono::system_clock::from_time_t(static_cast<time_t>(timestamp)) + 
               std::chrono::microseconds(static_cast<long>(timestamp * 1000000) % 1000000);
    });
    
    // Bind Buffer class with Python datetime support
    py::class_<tf_lcm::Buffer>(m, "Buffer")
        .def(py::init<double>(), py::arg("cache_time") = 10.0)
        .def("set_transform", [](tf_lcm::Buffer& self, py::object pyTransform, const std::string& authority, bool is_static) {
            // Convert Python transform to C++ format
            geometry_msgs::TransformStamped cppTransform = convertPyTransformToCpp(pyTransform);
            self.setTransform(cppTransform, authority, is_static);
        }, py::arg("transform"), py::arg("authority"), py::arg("is_static") = false)
        .def("set_transforms", [](tf_lcm::Buffer& self, py::list pyTransforms, const std::string& authority, bool is_static) {
            // Convert list of Python transforms to C++ format
            std::vector<geometry_msgs::TransformStamped> cppTransforms;
            for (auto item : pyTransforms) {
                cppTransforms.push_back(convertPyTransformToCpp(py::reinterpret_borrow<py::object>(item)));
            }
            self.setTransforms(cppTransforms, authority, is_static);
        }, py::arg("transforms"), py::arg("authority"), py::arg("is_static") = false)
        .def("lookup_transform", [](tf_lcm::Buffer& self, const std::string& target_frame, const std::string& source_frame, py::object time_obj, double timeout_seconds, py::object lcm_module) {
            // Convert Python datetime to C++ time_point
            auto time_point = std::chrono::system_clock::from_time_t(static_cast<time_t>(py::cast<double>(time_obj.attr("timestamp")()))) + 
                              std::chrono::microseconds(static_cast<long>(py::cast<double>(time_obj.attr("timestamp")()) * 1000000) % 1000000);
                              
            // Convert timeout to C++ duration
            auto timeout = std::chrono::duration<double>(timeout_seconds);
            
            // Look up the transform (C++ version)
            geometry_msgs::TransformStamped cppTransform = self.lookupTransform(target_frame, source_frame, time_point, timeout);
            
            // Convert to Python and return
            return convertCppTransformToPy(cppTransform, lcm_module);
        }, py::arg("target_frame"), py::arg("source_frame"), py::arg("time"), py::arg("timeout") = 0.0, py::arg("lcm_module") = py::none())
        .def("lookup_transform_with_fixed_frame", [](tf_lcm::Buffer& self, const std::string& target_frame, py::object target_time_obj, 
                                                 const std::string& source_frame, py::object source_time_obj, const std::string& fixed_frame, 
                                                 double timeout_seconds, py::object lcm_module) {
            // Convert Python datetimes to C++ time_points
            auto target_time = std::chrono::system_clock::from_time_t(static_cast<time_t>(py::cast<double>(target_time_obj.attr("timestamp")()))) + 
                              std::chrono::microseconds(static_cast<long>(py::cast<double>(target_time_obj.attr("timestamp")()) * 1000000) % 1000000);
            auto source_time = std::chrono::system_clock::from_time_t(static_cast<time_t>(py::cast<double>(source_time_obj.attr("timestamp")()))) + 
                              std::chrono::microseconds(static_cast<long>(py::cast<double>(source_time_obj.attr("timestamp")()) * 1000000) % 1000000);
                              
            // Convert timeout to C++ duration
            auto timeout = std::chrono::duration<double>(timeout_seconds);
            
            // Look up the transform (C++ version)
            geometry_msgs::TransformStamped cppTransform = self.lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame, timeout);
            
            // Convert to Python and return
            return convertCppTransformToPy(cppTransform, lcm_module);
        }, py::arg("target_frame"), py::arg("target_time"), py::arg("source_frame"), py::arg("source_time"), py::arg("fixed_frame"), py::arg("timeout") = 0.0, py::arg("lcm_module") = py::none())
        .def("can_transform", [](tf_lcm::Buffer& self, const std::string& target_frame, const std::string& source_frame, py::object time_obj, py::object error_msg_obj) {
            // Convert Python datetime to C++ time_point
            auto time_point = std::chrono::system_clock::from_time_t(static_cast<time_t>(py::cast<double>(time_obj.attr("timestamp")()))) + 
                              std::chrono::microseconds(static_cast<long>(py::cast<double>(time_obj.attr("timestamp")()) * 1000000) % 1000000);
            
            // Handle error_msg parameter
            std::string* error_msg_ptr = nullptr;
            std::string error_msg_storage;
            if (!error_msg_obj.is_none()) {
                error_msg_ptr = &error_msg_storage;
            }
            
            // Call the C++ function
            bool result = self.canTransform(target_frame, source_frame, time_point, error_msg_ptr);
            
            // Update error_msg if provided
            if (error_msg_ptr) {
                error_msg_obj = py::cast(error_msg_storage);
            }
            
            return result;
        }, py::arg("target_frame"), py::arg("source_frame"), py::arg("time"), py::arg("error_msg") = py::none())
        .def("can_transform_with_fixed_frame", [](tf_lcm::Buffer& self, const std::string& target_frame, py::object target_time_obj, 
                                              const std::string& source_frame, py::object source_time_obj, const std::string& fixed_frame, py::object error_msg_obj) {
            // Convert Python datetimes to C++ time_points
            auto target_time = std::chrono::system_clock::from_time_t(static_cast<time_t>(py::cast<double>(target_time_obj.attr("timestamp")()))) + 
                              std::chrono::microseconds(static_cast<long>(py::cast<double>(target_time_obj.attr("timestamp")()) * 1000000) % 1000000);
            auto source_time = std::chrono::system_clock::from_time_t(static_cast<time_t>(py::cast<double>(source_time_obj.attr("timestamp")()))) + 
                              std::chrono::microseconds(static_cast<long>(py::cast<double>(source_time_obj.attr("timestamp")()) * 1000000) % 1000000);
            
            // Handle error_msg parameter
            std::string* error_msg_ptr = nullptr;
            std::string error_msg_storage;
            if (!error_msg_obj.is_none()) {
                error_msg_ptr = &error_msg_storage;
            }
            
            // Call the C++ function
            bool result = self.canTransform(target_frame, target_time, source_frame, source_time, fixed_frame, error_msg_ptr);
            
            // Update error_msg if provided
            if (error_msg_ptr) {
                error_msg_obj = py::cast(error_msg_storage);
            }
            
            return result;
        }, py::arg("target_frame"), py::arg("target_time"), py::arg("source_frame"), py::arg("source_time"), py::arg("fixed_frame"), py::arg("error_msg") = py::none())
        .def("get_all_frame_names", &tf_lcm::Buffer::getAllFrameNames)
        .def("clear", &tf_lcm::Buffer::clear);

    // Bind TransformBroadcaster class with Python type conversion
    py::class_<tf_lcm::TransformBroadcaster>(m, "TransformBroadcaster")
        .def(py::init<>())
        .def("send_transform", [](tf_lcm::TransformBroadcaster& self, py::object pyTransform) {
            // Convert Python transform to C++ format and send it
            geometry_msgs::TransformStamped cppTransform = convertPyTransformToCpp(pyTransform);
            self.sendTransform(cppTransform);
        }, py::arg("transform"))
        .def("send_transforms", [](tf_lcm::TransformBroadcaster& self, py::list pyTransforms) {
            // Convert list of Python transforms to C++ format
            std::vector<geometry_msgs::TransformStamped> cppTransforms;
            for (auto item : pyTransforms) {
                cppTransforms.push_back(convertPyTransformToCpp(py::reinterpret_borrow<py::object>(item)));
            }
            self.sendTransform(cppTransforms);
        }, py::arg("transforms"));

    // Bind StaticTransformBroadcaster class with Python type conversion
    py::class_<tf_lcm::StaticTransformBroadcaster>(m, "StaticTransformBroadcaster")
        .def(py::init<>())
        .def("send_transform", [](tf_lcm::StaticTransformBroadcaster& self, py::object pyTransform) {
            // Convert Python transform to C++ format and send it
            geometry_msgs::TransformStamped cppTransform = convertPyTransformToCpp(pyTransform);
            self.sendTransform(cppTransform);
        }, py::arg("transform"))
        .def("send_transforms", [](tf_lcm::StaticTransformBroadcaster& self, py::list pyTransforms) {
            // Convert list of Python transforms to C++ format
            std::vector<geometry_msgs::TransformStamped> cppTransforms;
            for (auto item : pyTransforms) {
                cppTransforms.push_back(convertPyTransformToCpp(py::reinterpret_borrow<py::object>(item)));
            }
            self.sendTransform(cppTransforms);
        }, py::arg("transforms"));

    // Bind LCM class for Python - make sure the class name matches what's expected in Python
    py::class_<PyLCM>(m, "LCM")
        .def(py::init<>())
        .def(py::init<const std::string&>(), py::arg("url"))
        .def("handle", &PyLCM::handle)
        .def("handle_timeout", &PyLCM::handleTimeout, py::arg("timeout_ms"))
        .def("good", &PyLCM::good);

    // Bind TransformListener class with proper LCM support
    py::class_<tf_lcm::TransformListener>(m, "TransformListener")
        .def(py::init<double>(), py::arg("buffer_size") = 10.0)
        .def(py::init<tf_lcm::Buffer&>(), py::arg("buffer"))
        .def(py::init([](PyLCM& lcm, tf_lcm::Buffer& buffer) {
            // Convert PyLCM to std::shared_ptr<lcm::LCM>
            std::shared_ptr<lcm::LCM> lcm_ptr(&lcm, [](lcm::LCM*) {});
            return new tf_lcm::TransformListener(lcm_ptr, buffer);
        }), py::arg("lcm"), py::arg("buffer"))
        .def("get_buffer", static_cast<tf_lcm::Buffer& (tf_lcm::TransformListener::*)()>(&tf_lcm::TransformListener::getBuffer), 
             py::return_value_policy::reference_internal);
}
