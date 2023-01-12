cmake_minimum_required(VERSION 3.5)

project(TheSloppyWorld VERSION 0.6 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_PREFIX_PATH ${CMAKE_SOURCE_DIR}/lib)

find_package(Eigen3 REQUIRED)

find_package(Qt NAMES Qt6)
find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGL OpenGLWidgets)

set(PROJECT_SOURCES
    src/main.cpp
    src/AbstractOpenGLSample.h
    src/AbstractOpenGLSample.cpp
    src/OpenGLSampleFactory.h
    src/OpenGLSampleFactory.cpp
    src/samples/SinglePointSample.h
    src/samples/SinglePointSample.cpp
    src/samples/SingleTriangleSample.h
    src/samples/SingleTriangleSample.cpp
    src/samples/TriangleWithShaderSample.h
    src/samples/TriangleWithShaderSample.cpp
    src/samples/TriangleWithShaderAndRuntimeAttrib.h
    src/samples/TriangleWithShaderAndRuntimeAttrib.cpp
    src/samples/IndexDrawingSample.h
    src/samples/IndexDrawingSample.cpp
    src/samples/ClumsyScene.h
    src/samples/ClumsyScene.cpp
)

qt_add_executable(${PROJECT_NAME}
    MANUAL_FINALIZATION
    ${PROJECT_SOURCES}
)

target_link_libraries(${PROJECT_NAME}
PRIVATE
    Qt6::Widgets
    Qt6::OpenGLWidgets
    Qt6::OpenGL
    Eigen3::Eigen)

install(TARGETS ${PROJECT_NAME}
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

qt_finalize_executable(${PROJECT_NAME})
