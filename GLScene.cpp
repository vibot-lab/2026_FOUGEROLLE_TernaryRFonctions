/**
 * @file GLScene.cpp
 * @author Yohan FOUGEROLLE
 * @date 2026
 * * @brief This file is part of a research project on Generalized R-functions
 * for N-dimensional implicit modeling.
 * * @details Licensed under the MIT License.
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 */


#include "GLScene.h"
#include <GL/glut.h>
#include <algorithm>
#include <cmath>
#include "AppController.h" // needed to attach AppController functions to opengl callbacks

 // =========================================================================
 // CONSTRUCTOR & SYSTEM INITIALIZATION
 // =========================================================================

GLScene::GLScene()
    : lights_on(false),
    global_min(1e10),
    global_max(-1e10),
    is_rotating(false),
    window_width(1280),
    window_height(720)
{
    // Initialize default viewpoint spatial translation and tracking parameters
    Object_Move = Eigen::Vector3d(0.0, 0.0, 0.0);
    rotation_matrix = Eigen::Matrix4d::Identity();

    // -----------------------------------------------------------------
    //  CORE RESEARCH FRAMEWORK INITIALIZATION
    // -----------------------------------------------------------------

    // 1. Instantiate the implicit scene logical composition evaluator manager.
    // Initialized by default to true ==> Intersection mode.
    // Encapsulated within an explicit std::unique_ptr ensuring strict heap lifecycle safety.
    m_CadLogic = std::make_unique<CAD_Scene>(true);

    // 2. Instantiate the localized Marching Cubes simplicial isosurface sampler
    m_MCGenerator = std::make_unique<MarchingCubes>();
}

/**
 * @brief Initializes the GLUT windowing framework and configures the core OpenGL context.
 * @details Establishes the rendering environment by creating the hardware context,
 * mapping interactive static callbacks, and setting up initial material, lighting,
 * and anti-aliasing hardware states.
 * @note CRITICAL SEQUENCE: Hardware-specific gl* commands and GLUT callback registrations
 * are strictly executed AFTER glutCreateWindow to prevent context binding failures.
 * @param argc Reference to the command-line argument count.
 * @param argv Command-line argument vector array.
 */
void GLScene::Init(int& argc, char** argv) {

    // Phase 1: Set up window creation parameters and surface buffer requirements
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(window_width, window_height);

    // Phase 2: Instantiate the window and bind the live hardware OpenGL rendering context
    glutCreateWindow("R-Functions Research Sandbox");

    // Phase 3: Register global static processing wrappers to handle interactive events
    // (Must follow window instantiation so GLUT can map them onto the active window ID)
    glutDisplayFunc(AppController::DisplayCallback);
    glutReshapeFunc(AppController::ReshapeCallback);
    glutKeyboardFunc(AppController::KeyboardCallback);
    glutMouseFunc(AppController::MouseCallback);
    glutMotionFunc(AppController::MotionCallback);

    // Phase 4: Configure standard state hardware context defaults
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // Clean white viewport backing background
    glEnable(GL_DEPTH_TEST);              // Enable hardware depth-buffer comparisons

    // Trigger the pre-allocated illumination and material parameters setup
    setLights();

    // Phase 5: Establish anti-aliasing blending properties managing smooth contour tracing loops
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    //camera position and target
    camPos = Eigen::Vector3d(0,0,5);
    camTarget = Eigen::Vector3d(0, 0, 0);
}

// =========================================================================
// LIGHTING AND MATERIAL PROPERTY SPECIFICATIONS
// =========================================================================

/**
 * @brief Configures the hardware lighting environment and surface material properties.
 * @details Sets up a directional light source mimicking infinite solar rays and defines
 * the bidirectional reflectance properties for the compiled solid mesh geometries.
 * Enables dynamic color tracking and hardware normal vector normalization to safeguard
 * shading rendering stability during interactive camera operations.
 */
void GLScene::setLights() {
    // Phase 1: Enable global illumination hardware subsystems
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST); // Crucial guard ensuring proper z-buffer occlusions across overlapping primitives

    // Phase 2: Define Light 0 attributes (Directional source, infinite distance layout)
    GLfloat ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f }; // Soft structural fill ambient illumination
    GLfloat diffuse[] = { 0.8f, 0.8f, 0.8f, 1.0f }; // Bright scattering diffuse reflection component
    GLfloat specular[] = { 1.0f, 1.0f, 1.0f, 1.0f }; // Crisp white specular highlight reflection spots
    GLfloat position[] = { 1.0f, 1.0f, 1.0f, 0.0f }; // W=0.0 denotes an infinite directional vector configuration

    // doubles side face lighting
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE); 

    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
    glLightfv(GL_LIGHT0, GL_POSITION, position);

    // Phase 3: Set default solid geometry surface material properties
    GLfloat mat_diffuse[] = { 0.7f, 0.7f, 0.7f, 1.0f };
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);

    // Phase 4: Enable Color-Material Tracking Pipeline Synchronization
    // Binds the active glColor3dv() stream directly onto ambient and diffuse material attributes.
    // This allows dynamic colormap shading (discrete potential fields scalar fields) 
    // to remain fully operational while keeping the lighting and specular calculations active.
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

    // Phase 5: Force automatic normal vector normalization rescale
    // Critical safeguard maintaining shading intensity stability. Prevents the lighting model 
    // from darkening or burning out when spatial transformations or trackball scaling operations 
    // alter the magnitude of geometric normal vectors.
    glEnable(GL_NORMALIZE);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
}

// =========================================================================
// VALUE DOMAIN BOUNDS MONITORING
// =========================================================================

void GLScene::UpdateRange(double val) {
    // Dynamic runtime mapping tracking scalar bounds across potential field evaluation steps
    if (val < global_min) global_min = val;
    if (val > global_max) global_max = val;
}

// =========================================================================
// SCIENTIFIC RENDERING & HIGH-LEVEL GEOMETRIC VISUALIZATION
// =========================================================================

void GLScene::DrawAxes() {
    // Temporarily bypass illumination context parameters to render pure emission coloring lines
    glDisable(GL_LIGHTING);
    glLineWidth(3.0f);

    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f);  glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(1.0f, 0.0f, 0.0f); // X Axis (Red)
    glColor3f(0.0f, 0.7f, 0.0f);  glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(0.0f, 1.0f, 0.0f); // Y Axis (Dark green for high visibility)
    glColor3f(0.0f, 0.0f, 1.0f);  glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(0.0f, 0.0f, 1.0f); // Z Axis (Blue)
    glEnd();
}

void GLScene::RenderGlobalMesh(int listId) {
    // Call the static hardware display list block to bypass heavy repetitive data transfer states
    if (listId != -1 && glIsList(listId)) {
        glCallList(listId);
    }
}


// draws white circles (intersection with the spheres at z=0) to delimitate the boundaries
void GLScene::DrawSphereContour(const Eigen::Vector3d& center, double radius, const Eigen::Vector3d& color) {
    // -----------------------------------------------------------------
    // ANALYTICAL GEOMETRY: SPHERE-PLANE INTERSECTION CIRCLE
    // -----------------------------------------------------------------
    // Determines the intersection circle equation where the sphere intercepts the reference plane z = 0
    double h2 = center[2] * center[2]; // Squared height offset from the plane boundary
    double R2 = radius * radius;

    // Boundary check: if the projection height exceeds the radius, no intersection happens
    if (h2 >= R2) return;

    // Compute the exact radius of the intersection circle via Pythagorean extraction
    double r = std::sqrt(R2 - h2);
    int num_segments = 100;

    glDisable(GL_LIGHTING);
    glLineWidth(2.0f);
    glColor3d(color[0], color[1], color[2]);

    // Setup an isolated pattern mask tracking high-visibility dashed line render frames
    glEnable(GL_LINE_STIPPLE);
    glLineStipple(1, 0x00FF); // Alternate bitmask trace template sequence (stipple pattern)

    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < num_segments; i++) {
        // Parametric circular coordinate tracking steps formula
        double theta = 2.0 * M_PI * double(i) / double(num_segments);
        double x = center[0] + r * std::cos(theta);
        double y = center[1] + r * std::sin(theta);
        glVertex3d(x, y, 0.0); // Project coordinates flatly onto the target z=0 plane cut
    }
    glEnd();

    glDisable(GL_LINE_STIPPLE);
}

// now unused I guess
void GLScene::RenderGradientGlyphs(
    const std::vector<Eigen::Vector3d>& points,
    const std::vector<Eigen::VectorXd>& gradients,
    const Eigen::Vector3d& color,
    double scale)
{
    // Bypasses lighting models to plot high-accuracy directional line tracks vectors
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);

    for (size_t i = 0; i < points.size(); ++i) {
        Eigen::Vector3d P = points[i];
        Eigen::Vector3d G = gradients[i].head<3>(); // Isolate and project spatial 3D components

        // -----------------------------------------------------------------
        // ANISOTROPIC QUALITY ANALYSIS COLOR FIELD LOOKUP
        // -----------------------------------------------------------------
        // Color mapping diagnostics metric: a unit normal norm magnitude (|G| = 1) implies perfect 
        // field stability properties, whereas deviation highlights potential local gradient issues.
        double mag = G.norm();
        if (std::abs(mag - 1.0) < 0.1) {
            glColor3f(0.0f, 1.0f, 0.0f); // Green color signature: validated unit gradient condition
        }
        else {
            glColor3f(1.0f, 0.0f, 0.0f); // Red color signature: field deviation or singularity artifact boundary
        }

        // Project coordinate vertices representing the directed gradient glyph track line segment
        glVertex3d(P.x(), P.y(), P.z());
        glVertex3d(P.x() + G.x() * scale, P.y() + G.y() * scale, P.z() + G.z() * scale);
    }
    glEnd();
    glEnable(GL_LIGHTING); // Restore the ambient hardware light system context matrices
}