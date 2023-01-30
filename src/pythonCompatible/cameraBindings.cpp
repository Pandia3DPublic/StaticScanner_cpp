#include "../CameraWrapper.h"
#include "VolumeCalculation.h"
#include <pybind11/stl.h>
#include "util.h"

namespace py = pybind11;
using namespace pybind11::literals;
using namespace std;
int add(int i, int j)
{
    // auto sphere = open3d::geometry::TriangleMesh::CreateSphere(1);
    // open3d::visualization::DrawGeometries({sphere});
    return i + j + g_test;
}




PYBIND11_MODULE(cameraBindings, m)
{

    py::enum_<CameraTypes>(m, "CameraTypes")
        .value("AzureKinect", AzureKinect)
        .value("IntelCamera", IntelCamera)
        .value("DataCamera", DataCamera)
        .value("PhoxiScanner", PhoxiScanner)
        .value("ZividCamera", ZividCamera)
        .value("EnsensoCamera", EnsensoCamera)
        .export_values();

    py::class_<CameraWrapper>(m, "CameraWrapper")
        .def(py::init<>())
        .def(py::init<CameraTypes>())
        .def("connect", &CameraWrapper::connect, py::arg("CameraNumber") = 0, py::arg("DataCamPath_") = "")
        .def("record", &CameraWrapper::record, "n"_a, "nwait"_a = 30, "checkDepthImage"_a = true)
        .def("showImages", &CameraWrapper::showOpen3dImages)
        .def("AlignUndistortandConvertO3D", &CameraWrapper::AlignUndistortandConvertO3D)
        .def("clearBuffers", &CameraWrapper::clearBuffers)
        .def("printInfo", &CameraWrapper::printInfo)
        .def("ReadFOVDataFromDisk", &CameraWrapper::ReadFOVDataFromDisk)
        .def("AlignUndistortConvert", &CameraWrapper::AlignUndistortandConvertO3D)
        .def("CropRGBDImages", &CameraWrapper::CropRGBDImages)
        .def("disconnect", &CameraWrapper::disconnect)
        .def_readwrite("FRCalibrationImages", &CameraWrapper::FRCalibrationImages)
        .def_readwrite("VolumeImage", &CameraWrapper::VolumeImage);

    py::class_<cv::Mat>(m, "cvMatrix", py::buffer_protocol())
        .def_buffer([](cv::Mat &m) -> py::buffer_info
                    {
                        string formatdescriptor;
                        if (m.elemSize1() == 2){
                            formatdescriptor = py::format_descriptor<short>::format();
                        } else if(m.elemSize1() == 1){
                            formatdescriptor = py::format_descriptor<uchar>::format();
                        }
                        return py::buffer_info(
                            m.data,                         /* Pointer to buffer */
                            m.elemSize1(),                  /* Size of one scalar */
                            formatdescriptor,               /* Python struct-style format descriptor */
                            3,                              /* Number of dimensions */
                            {m.rows, m.cols, m.channels()}, /* Buffer dimensions */
                            {m.elemSize1() * m.channels() * m.cols,
                             m.elemSize1() * m.channels(), /* Strides (in bytes) for each index */
                             m.elemSize1()});
                    });

    m.def("add", &add, "A function which adds two numbers");
    m.def("getCurrentVolume", &getCurrentVolume, "A function which returns the current volume");
    m.def("getNumberofConnectedDevices", &getNumberofConnectedDevices, "get the number of connected devices");
    m.def("readconfig", &readconfig, "Read the config file and set some variables");
    // m.def("converCVToArray", &getVectorFromImage, "convert cv to array");
}