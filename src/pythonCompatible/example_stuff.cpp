// #include "GlobalDefines.h"
// using namespace std;
// int add(int i, int j)
// {
//     // auto sphere = open3d::geometry::TriangleMesh::CreateSphere(1);
//     // open3d::visualization::DrawGeometries({sphere});
//     return i + j+ g_test;
// }

// namespace py = pybind11;



// class Dog{
// public:
//     Dog(){};
//     ~Dog(){};
//     string name;
//     int age;
//     int dogage;
// };

// int addDogAges(Dog& d){
//     return d.age + d.dogage;
// }


// PYBIND11_MODULE(example_module, m)
// {
//     m.def("add", &add, "A function which adds two numbers");

//     py::class_<Dog>(m, "Dog")
//         .def(py::init<>())
//         .def_readwrite("name", &Dog::name)
//         .def_readwrite("age", &Dog::age)
//         .def_readwrite("dogage", &Dog::dogage);

//     m.def("dogadd", &addDogAges, "A function which adds two dog numbers");
// }