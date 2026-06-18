cmake_minimum_required(VERSION 3.16)

project(TernaryRFunctions)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------

find_package(OpenGL REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(Boost REQUIRED)

# yaml
find_package(yaml-cpp REQUIRED)

# FreeGLUT
find_package(GLUT REQUIRED)

# ------------------------------------------------------------------------------
# Sources
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
# Executable
# ------------------------------------------------------------------------------

add_executable(${PROJECT_NAME}
    ${SOURCES}
    ${HEADERS}
)

# ------------------------------------------------------------------------------
# Link libraries
# ------------------------------------------------------------------------------

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        OpenGL::GL
        OpenGL::GLU
        GLUT::GLUT
        Eigen3::Eigen
        yaml-cpp
)