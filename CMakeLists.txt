cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 14)

CMAKE_POLICY(SET CMP0057 NEW) #todo what does that do? KOkkos rule launch relevant?
project(StaticScanner LANGUAGES CXX CUDA)

file(GLOB_RECURSE srcfiles 
src/*.h   
src/*.hpp
src/*.cpp
src/*.cu
)
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(SYSTEM	${PROJECT_SOURCE_DIR}/pchs)

#find lib packages here
find_package(k4a REQUIRED)
set(Open3D_DIR ${PROJECT_SOURCE_DIR}/3rdParty/Open3D/lib/cmake/Open3D)
find_package(Open3D HINTS Open3D_DIR REQUIRED)
set(OpenCV_DIR ${PROJECT_SOURCE_DIR}/3rdParty/opencv/lib/cmake/opencv4/)
find_package(OpenCV REQUIRED)
# Find librealsense2 installed package
find_package(realsense2 REQUIRED)
# find ogre for gui
set(OGRE_DIR ${PROJECT_SOURCE_DIR}/3rdParty/ogre/install/lib/OGRE/cmake)
find_package(OGRE 1.12.12 REQUIRED COMPONENTS Bites RTShaderSystem HINTS OGRE_DIR)
set(CUSTOM_RESOURCES_DIR ${PROJECT_SOURCE_DIR}/resources)
configure_file(${CUSTOM_RESOURCES_DIR}/ogre/configs/resources.cfg ${CMAKE_CURRENT_BINARY_DIR}/resources.cfg)
configure_file(${CUSTOM_RESOURCES_DIR}/ogre/packs/SdkTrays.zip ${OGRE_MEDIA_DIR}/packs/SdkTrays.zip COPYONLY)
find_package(OpenMP REQUIRED)
find_package(PhoXi REQUIRED CONFIG PATHS "$ENV{PHOXI_CONTROL_PATH}")
find_package(Zivid COMPONENTS Core REQUIRED)
find_package(pybind11 REQUIRED HINTS ${PROJECT_SOURCE_DIR}/3rdParty/pybind11/share/cmake)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "$ENV{ENSENSO_INSTALL}/cmake/")
find_package(Ensenso REQUIRED)
find_package(jsoncpp REQUIRED HINTS ${PROJECT_SOURCE_DIR}/3rdParty/jsoncpp/lib/cmake)
get_target_property(JSONCPP_INCLUDE_DIRS jsoncpp_lib INTERFACE_INCLUDE_DIRECTORIES)

#choose executable
if (${EXECUTABLE} STREQUAL "Test")
message("################### Building Test ###################")
set(target_name_exe Test) #cpp files with same name must be located in /exe 
endif()
if (${EXECUTABLE} STREQUAL "StaticScanner")
message("################### Building StaticScanner ###################")
set(target_name_exe StaticScanner) #cpp files with same name must be located in /exe 
endif()
if (${EXECUTABLE} STREQUAL "VolumeScanner")
message("################### Building VolumeScanner ###################")
set(target_name_exe VolumeScanner) #cpp files with same name must be located in /exe 
endif()
if (${EXECUTABLE} STREQUAL "SimpleRecorder")
message("################### Building SimpleRecorder ###################")
set(target_name_exe SimpleRecorder) #cpp files with same name must be located in /exe 
endif()

#add python libs to targets
# pybind11_add_module(cameraBindings src/pythonCompatible/cameraBindings.cpp)
# set(target_names ${target_name_exe}) #cpp files with same name must be located in exe
set(target_names ${target_name_exe}) #cpp files with same name must be located in exe
message("########################")
message(${target_names})
add_executable(${target_name_exe} exe/${target_name_exe}.cpp)
# add_executable(${target_name_exe} exe/${target_name_exe}.cpp)
# add_executable(${target_name_exe} exe/${target_name_exe}.cpp ${srcfiles})
FOREACH(target_name ${target_names})
    target_sources(${target_name} PRIVATE ${srcfiles})
    target_precompile_headers(${target_name} PUBLIC ${PROJECT_SOURCE_DIR}/pchs/pch.h)
    target_link_libraries(${target_name} PRIVATE k4a)
    target_include_directories(${target_name} PUBLIC ${Open3D_INCLUDE_DIRS})
    target_link_libraries(${target_name} PRIVATE ${Open3D_LIBRARIES})
    target_include_directories(${target_name} PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${target_name} PRIVATE ${OpenCV_LIBRARIES})
    target_link_libraries(${target_name} PRIVATE ${realsense2_LIBRARY})
    target_link_libraries(${target_name} PRIVATE OgreBites OgreRTShaderSystem)
    target_link_libraries(${target_name} PRIVATE OpenMP::OpenMP_CXX)
    target_include_directories (${target_name} PUBLIC ${PHOXI_INCLUDE_DIRS})
    target_link_libraries(${target_name} PRIVATE ${PHOXI_LIBRARY} $<$<PLATFORM_ID:Linux>:rt>)
    target_include_directories (${target_name} PUBLIC ${pybind11_INCLUDE_DIRS})
    target_link_libraries(${target_name} PRIVATE pybind11::pybind11 pybind11::embed pybind11::module) #gruselig TODO
    target_include_directories (${target_name} PUBLIC ${ZIVID_INCLUDE_DIRS})
    target_link_libraries(${target_name} PRIVATE Zivid::Core)
    target_include_directories (${target_name} PUBLIC ${ENSENSO_INCLUDE_DIRS})
    target_link_libraries(${target_name} PRIVATE ${ENSENSO_LIBRARIES})
    target_include_directories (${target_name} PUBLIC ${JSONCPP_INCLUDE_DIRS})
    target_link_libraries(${target_name} PRIVATE jsoncpp_lib)
ENDFOREACH()
# target_compile_definitions(${target_name} PRIVATE ${compileDefs})

#copy the config.txt
configure_file(config.txt ${CMAKE_CURRENT_BINARY_DIR} COPYONLY )