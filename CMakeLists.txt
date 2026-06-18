cmake_minimum_required(VERSION 3.16)

project(TernaryRFunctions)

# ------------------------------------------------------------------------------
# C++ Standard Configuration
# ------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ------------------------------------------------------------------------------
# Compiler-Specific Optimization Flags
# ------------------------------------------------------------------------------
if(MSVC)
    # Maximum optimization flags for Microsoft Visual C++ Compiler (MSVC)
    # /Ox: Enable maximum optimizations (favoring speed)
    # /Oi: Enable intrinsic functions
    # /Ot: Favor code speed over code size
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Ox /Oi /Ot")
else()
    # Maximum optimization flag for GCC and Clang compilers (Linux/macOS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

# ------------------------------------------------------------------------------
# Third-Party Dependencies
# ------------------------------------------------------------------------------

# Locate required system libraries
find_package(OpenGL REQUIRED)
find_package(Eigen3 REQUIRED)

# Configuration parser dependency
find_package(yaml-cpp REQUIRED)

# Windowing and rendering context utility
find_package(GLUT REQUIRED)

# ------------------------------------------------------------------------------
# Source and Header Files Definition
# ------------------------------------------------------------------------------
set(SOURCES
    AppConfig.cpp
    AppController.cpp
    GLScene.cpp
    ImplicitObjects.cpp
    MarchingCubes.cpp
    Mesh.cpp
    NeighborMesh.cpp
    Rfunctions.cpp
    main.cpp
)

set(HEADERS
    AppConfig.h
    AppController.h
    CAD_Scene.h
    GLScene.h
    ImplicitObjects.h
    MarchingCubes.h
    Mesh.h
    NeighborMesh.h
    Rfunctions.h
)

# ------------------------------------------------------------------------------
# Executable Target Generation
# ------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}
    ${SOURCES}
    ${HEADERS}
)

# ------------------------------------------------------------------------------
# Target Link Libraries & Dependency Bindings
# ------------------------------------------------------------------------------
target_link_libraries(${PROJECT_NAME}
    PRIVATE
        OpenGL::GL
        OpenGL::GLU
        GLUT::GLUT
        Eigen3::Eigen
        yaml-cpp
)