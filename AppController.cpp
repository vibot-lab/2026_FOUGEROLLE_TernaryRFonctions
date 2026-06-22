/**
 * @file AppController.cpp
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

#include "AppController.h"
#include <GL/glut.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <random>
#include <cmath>
#include <algorithm>

 // Initialize the static singleton instance pointer to secure global framework callbacks
AppController* AppController::s_instance = nullptr;

// =========================================================================
// CONSTRUCTOR, LIFECYCLE MANAGEMENT & INITIALIZATION
// =========================================================================

AppController::AppController()
    : m_scene(),             // 1. Instantiates the global GLScene context managing rendering parameters
    m_functionUsed(0),     // 2. Default target index pointer for R-function selection arrays
    m_evaluator(),         // 3. Instantiates the Spherical simplex workspace dedicated to ternary formulations
    m_activeMeshId(-1),    // 4. Initial state: no active OpenGL display list compiled yet
    m_isIntersection(true),// 5. Logical composition toggle default: true = Intersection, false = Union
    m_is3D(false),         // 6. Viewport default representation: flat 2D projection mapping
    m_sampling(500)        // 7. Planar grid discretization density index (Total vertex map = (sampling+1)^2)
{
    s_instance = this;
    SpherEval.Init(3);       // Spherical continuous logical operators are strictly bounded to 3D spaces
}


//creates the 3 spheres as in the paper
void AppController :: makeSpheres(){

    m_objects.clear();
    m_objects.push_back(std::make_shared<ImplicitSphere>(Eigen::Vector3d(-0.5, -0.5, 0.0), 1.0)); // Sphere 1
    m_objects.push_back(std::make_shared<ImplicitSphere>(Eigen::Vector3d(0.5, -0.5, 0.0), 0.8));  // Sphere 2
    m_objects.push_back(std::make_shared<ImplicitSphere>(Eigen::Vector3d(0.0, 0.5, 0.0), 1.2));  // Sphere 3
}

void AppController::Init(int& argc, char** argv) {

    // OpenGL initialisation and callback attachement
    m_scene.Init(argc, argv);

    // Procedural scene populating paths depending on the current  mode configuration

    // classical example mode ==> simple implicit scene (3 spheres or a regular polygon)
    if (currentMode == SceneMode::ClassicalExample)
    {
        // 3 spheres as in the CAD paper
        makeSpheres();

        // regular polygon.
        int n = 6; double R = 1.25;
        //makePolygon(n, R);
    }

    if (currentMode == SceneMode::BenchmarkGradient)
    {
        // Reserved for dynamic field sampling tracking setups
    }

    if (currentMode == SceneMode::BenchmarkGradientUnbalancedTree)
    {
        // Assembles a 9-sphere symmetrical daisy pattern to evaluate deep composition cost trees
        m_objects = MakeDaisy(1,0.5,9);
    }

    if (currentMode == SceneMode::NaryExample)
    {
        std::cout << "[N-ary Example] - Daisy pattern construction" << std::endl;
        m_objects = MakeDaisy(0.75, 1.0, 7);
    }
}

void AppController::Run() {
    // Launch interactive OpenGL/GLUT application main loop execution
    glutMainLoop();
}

// =========================================================================
// MATHEMATICAL EVALUATION & ALGEBRAIC RECURSIVE ENGINES
// =========================================================================

// Hardcoded ternary decomposition for exactly 9 primitives (e.g., the Daisy flower scene).
// WARNING: This specific implementation assumes X contains 9 elements 
// and bubbles up 1 recursive level to compute the final composite potential.

double AppController::EvaluateRecursiveSpherical(const Eigen::VectorXd& X, bool inter, RCore::SphericalEvaluator& eval) {
    int n = static_cast<int>(X.size());

    // Base conditions: direct evaluation if dimension constraints are satisfied
    if (n == 3) return eval.Compute(inter, X);
    if (n == 1) return X[0];

    // Tree decomposition path: subdivide the higher-order N-vector into standard ternary segments
    int nextSize = n / 3;
    Eigen::VectorXd nextLevel(nextSize);
    for (int i = 0; i < nextSize; ++i) {
        nextLevel[i] = eval.Compute(inter, X.segment<3>(i * 3));
    }

    // Bubble up recursively until reaching the root operator index
    return EvaluateRecursiveSpherical(nextLevel, inter, eval);
}


/**
 * @brief Evaluates and samples the comprehensive potential field matrix across the entire parametric 2D grid domain.
 * @details High-performance implementation optimized to mitigate cache fragmentation and memory allocation overhead.
 * Leverages OpenMP parallel loop distribution and eliminates internal heap allocations by replacing
 * runtime nested std::vector structures with compile-time fixed stack arrays.
 * @param sampling Grid discretization step density density (Total grid size computed = (sampling + 1)^2).
 * @return std::vector<std::pair<Eigen::Vector3d, std::vector<Eigen::Vector2d>>> Flattened data container storing
 * spatial positions paired with their multi-logic analytical R-function evaluations array.
 */
std::vector<std::pair<Eigen::Vector3d, std::vector<Eigen::Vector2d>>>
AppController::ComputeFullGridData(unsigned int sampling)
{
    assert(sampling <= 1000); // Safety guard preventing stack heap explosion allocations

    const auto& dom = m_scene.m_domain;
    const size_t total_points = static_cast<size_t>(sampling + 1) * static_cast<size_t>(sampling + 1);

    // Allocate the global container contiguously in memory upfront
    std::vector<std::pair<Eigen::Vector3d, std::vector<Eigen::Vector2d>>> data(total_points);

    const int m = Rvachev_m;
    const size_t num_primitives = m_objects.size();

    // -----------------------------------------------------------------
    // PARALLELIZED MULTI-THREADED SAMPLING ENGINE (OpenMP)
    // -----------------------------------------------------------------
    // Automatically distributes spatial row calculations across all available hardware thread pools.
    // Thread safety is achieved by tracking and isolating variable instantiations on individual thread stacks.
#pragma omp parallel
    {
        // Allocate a thread-local field vector map mapping object potentials to avoid concurrent reallocations
        Eigen::VectorXd X_local(num_primitives);

        // Pre-allocate a fixed stack-based array layout to hold evaluations without calling the heap allocator
        std::vector<Eigen::Vector2d> RFarray_local(RFnumber);

#pragma omp for schedule(guided)
        for (int j = 0; j <= static_cast<int>(sampling); ++j) {
            double v_coord = double(j) / sampling;
            size_t row_offset = static_cast<size_t>(j) * (sampling + 1);

            for (unsigned int i = 0; i <= sampling; ++i) {
                double u_coord = double(i) / sampling;
                size_t global_idx = row_offset + i;

                // Map current parametric step coordinates onto explicit 3D spaces
                Eigen::Vector3d P = dom.LC + dom.scale * (u_coord * dom.U + v_coord * dom.V);

                // Populate potentials across all analytical primitives into thread-isolated buffers
                for (size_t k = 0; k < num_primitives; ++k) {
                    X_local[k] = m_objects[k]->Evaluate(P);
                }

                // --- LINEAR OPERATOR EVALUATION STACK ---

                // Row 0: Classical non-differentiable Min/Max continuous boundaries logic
                RFarray_local[0][0] = X_local.minCoeff();
                RFarray_local[0][1] = X_local.maxCoeff();

                // Row 1: Binary Rp=2 operators chained recursively on an unbalanced peigne layout
                double val_inter = X_local[0];
                double val_union = X_local[0];
                for (Eigen::Index k = 1; k < X_local.size(); ++k) {
                    val_inter = RCore::Rp(val_inter, X_local[k], true);
                    val_union = RCore::Rp(val_union, X_local[k], false);
                }
                RFarray_local[1][0] = val_inter;
                RFarray_local[1][1] = val_union;

                // Row 2: Same with R0m
                val_inter = X_local[0];
                val_union = X_local[0];
                for (Eigen::Index k = 1; k < X_local.size(); ++k) {
                    val_inter = RCore::R0m(val_inter, X_local[k], true, m);
                    val_union = RCore::R0m(val_union, X_local[k], false, m);
                }
                RFarray_local[2][0] = val_inter;
                RFarray_local[2][1] = val_union;

                // Row 3: Zenkin direct N-ary 
                RFarray_local[3][0] = RCore::ZenkinNd(X_local, true);
                RFarray_local[3][1] = RCore::ZenkinNd(X_local, false);

                // Row 4: Rvachev direct N-ary
                RFarray_local[4][0] = RCore::RvachevNd(X_local, true);
                RFarray_local[4][1] = RCore::RvachevNd(X_local, false);

                // Row 5: Proposed directional spherical model (strictly requires dimension 3)
                if (X_local.size() == 3) {
                    RFarray_local[5][0] = SpherEval.Compute(true, X_local);
                    RFarray_local[5][1] = SpherEval.Compute(false, X_local);
                }
                else {
                    RFarray_local[5][0] = RFarray_local[5][1] = 0.0;
                }

                // Row 6: Normalized directional gradient variant preserving field stability
                if (X_local.size() == 3) {
                    RFarray_local[6][0] = SpherEval.ComputeNormalized(true, X_local);
                    RFarray_local[6][1] = SpherEval.ComputeNormalized(false, X_local);
                }
                else {
                    RFarray_local[6][0] = RFarray_local[6][1] = 0.0;
                }


                // Write the results directly into the globally structured vector block using fast stack assignments
                data[global_idx].first = P;
                data[global_idx].second = RFarray_local;
            }
        }
    } // End of OpenMP parallel block execution space

    return data;
}

std::vector<std::shared_ptr<ImplicitObject>> AppController::MakeDaisy(double R, double Rc, double num) {
    
    std::vector<std::shared_ptr<ImplicitObject>> daisy;

    // Equi-angular structural placement distributing centers symmetrically along a planar tracking circle
    for (int i = 0; i < num; ++i) {
        double angle = 2.0 * M_PI * i / num;
        daisy.push_back(std::make_shared<ImplicitSphere>(Eigen::Vector3d(Rc * std::cos(angle), Rc * std::sin(angle), 0.0), R ));
    }
    return daisy;
}

std::vector<std::shared_ptr<ImplicitObject>> AppController::MakeDaisyConvexTest(
    double& epsilonCrit,
    int primitiveNumber,
    double R,
    double Rc
) {
    std::vector<std::shared_ptr<ImplicitObject>> daisy;
    int n = std::max(1, primitiveNumber);

    // Standard engine tracking initialization utilizing constant seed bounds to secure tracking consistency
    static std::mt19937 gen(42);

    std::uniform_real_distribution<double> distAngle(-0.3, 0.3);
    std::uniform_real_distribution<double> distVar(0.5, 1.5);

    double baseAngleStep = (2.0 * M_PI) / static_cast<double>(n);
    double maxEps = 0.0;

    for (int i = 0; i < n; ++i) {
        // 1. Structural anisotropic angular offset placement
        double angle = (i * baseAngleStep) + distAngle(gen);
        double currentD = R * distVar(gen);

        Eigen::Vector3d center(currentD * std::cos(angle), currentD * std::sin(angle), 0.0);

        // 2. Guarantee structural intersection overlap conditions enclosing coordinate origins
        double minRadius = center.norm();
        double finalRadius = std::max(minRadius + 0.6 * Rc, Rc * distVar(gen));

        daisy.push_back(std::make_shared<ImplicitSphere>(center, finalRadius));

        // 3. Compute local analytical mathematical bounds constraints for convexity modeling
        // Gamma factor = 2.0 parameterizes quadratic derivative classifications boundaries
        double gamma = 2.0;

        // Numerical safety guard preventing division operations by zero boundaries near the origin
        double localEps = (currentD > 1e-6) ? (1.0 / (currentD * std::sqrt(gamma))) : 10.0;

        if (localEps > maxEps) maxEps = localEps;
    }

    epsilonCrit = maxEps;
    std::cout << "epsilonCrit dans MakeDaisyConvexTest=" << epsilonCrit << std::endl;

    return daisy;
}


// =========================================================================
// INTERACTIVE GRAPHICS ENGINE RENDERING & OPENGL BINDING BUFFERS
// =========================================================================

 void AppController::ComputeAndBufferMesh(unsigned int sampling) {

     if (currentMode != SceneMode::CADjunction)
     {
         // 1. Core structural field potentials computations passes
         
         if (currentMode == SceneMode::NaryExample || currentMode == SceneMode::ClassicalExample)
         {
             std::cout << "\t --Grid sampling density: "
                 << sampling << "x" << sampling << "\n";
             data = ComputeFullGridData(sampling);
         }

         if (data.empty()) return;

         m_scene.local_min_max.resize(RFnumber);

         // 2. Synchronize spatial range color mapping parameters tables
         m_scene.ResetRange();
         m_scene.local_min_max.assign(RFnumber, Eigen::Vector2d(1e10, -1e10));

         for (const auto& p : data) {
             for (int f = 0; f < RFnumber; ++f) {
                 for (int op = 0; op < 2; ++op) {
                     double val = p.second[f][op];

                     // Refine local analytical limits scales per logic method
                     if (val < m_scene.local_min_max[f][0]) m_scene.local_min_max[f][0] = val;
                     if (val > m_scene.local_min_max[f][1]) m_scene.local_min_max[f][1] = val;

                     // Synchronize absolute global maximum limits values
                     if (val < m_scene.global_min) m_scene.global_min = val;
                     if (val > m_scene.global_max) m_scene.global_max = val;
                 }
             }
         }

         // 3. Purge historical stale display lists to protect graphics memory structures
         for (auto& vec : m_scene.RF_ogl_lists) {
             if (vec[0] > 0) glDeleteLists(vec[0], 1);
             if (vec[1] > 0) glDeleteLists(vec[1], 1);
         }
         m_scene.RF_ogl_lists.assign(RFnumber, Eigen::Vector2i::Zero());

         // 4. Drive structural geometric discrete elements compiler buffers
         unsigned int stride = sampling + 1;

         for (int funcIdx = 0; funcIdx < RFnumber; ++funcIdx) {
             for (int opIdx = 0; opIdx < 2; ++opIdx) {
                 GLuint listId = glGenLists(1);
                 m_scene.RF_ogl_lists[funcIdx][opIdx] = (int)listId;

                 glNewList(listId, GL_COMPILE);
                 glBegin(GL_QUADS); // Process quad primitive grid patches

                 for (unsigned int y = 0; y < sampling; ++y) {
                     for (unsigned int x = 0; x < sampling; ++x) {
                         unsigned int idx[4] = {
                             y * stride + x,
                             (y + 1) * stride + x,
                             (y + 1) * stride + (x + 1),
                             y * stride + (x + 1)
                         };

                         for (int k = 0; k < 4; ++k) {
                             const auto& pointData = data[idx[k]];
                             double val = pointData.second[funcIdx][opIdx];
                             const Eigen::Vector3d& pos = pointData.first;

                             // Map continuous local fields values into discrete color indices maps step layouts
                             Eigen::Vector3d col = doubleToColorDiscreteUpdatedLocal(val, 15, m_scene.local_min_max[funcIdx][0], m_scene.local_min_max[funcIdx][1]);
                             glColor3dv(col.data());

                             // Viewport perspective mapping projection: project potential field value as Z elevation if m_is3D is active
                             double zValue = m_is3D ? val : 0.0;
                             glVertex3d(pos.x(), pos.y(), zValue);
                         }
                     }
                 }
                 glEnd();
                 glEndList();
             }
         }

         std::cout << "\t --Successfully buffered " << RFnumber * 2 << " OpenGL lists.\n" << std::endl;
     }

     else {
         std::cout << "[CAD JUNCTION MODE]:\n";
         std::cout << "Starting Mesh buffering..." << std::endl;
         {

             if (Mesh.vertices.empty() || Mesh.faces.empty())
             {
                 std::cerr << "[ERROR] Mesh not initialized. Run dedicated main program to :\n"
                     << "\t --Run Marching Cube Algorithm\n"
                     << "\t --Clean soup triangles to make watertight Mesh\n"
                     << "\t --Adjust OpenGL display list and keyboard behavior\n\n";

                 return;
             }
             // Allocation step initializing 3D simplicial multi-junction geometry models rendering maps
             m_scene.RF_ogl_lists.assign(RFnumber, Eigen::Vector2i::Zero());

             GLuint listId = glGenLists(1);
             m_scene.RF_ogl_lists[0][0] = (int)listId;

             glNewList(listId, GL_COMPILE);

             Mesh.DrawBoundaryEdges();
             Mesh.Draw(FACE_NORMAL);

             glEndList();
         }
     }
 }


void AppController::updateActiveListIndex() {
    if (m_activeMethod >= 0 && m_activeMethod < RFnumber) {
        // Map logical composition modes onto discrete structure positions (Intersection -> 0, Union -> 1)
        int opIdx = m_isIntersection ? 0 : 1;

        // Synchronize active targeted list index IDs
        active_list_idx = m_scene.RF_ogl_lists[m_activeMethod][opIdx];

        std::string opName = m_isIntersection ? "INTERSECTION" : "UNION";

        if (currentMode == AppController::SceneMode::ClassicalExample
            ||
            currentMode == AppController::SceneMode::NaryExample
            ) {
            std::cout << "[RENDER] Method " << m_activeMethod
                << ": " << getFunctionName(m_activeMethod)
                << " | Op: " << opName
                << " | OpenGL List ID: " << active_list_idx << std::endl;
        }
    }
    else {
        std::cerr << "[ERROR] m_activeMethod (" << m_activeMethod << ") out of range!" << std::endl;
    }
}


// =========================================================================
// INTERACTIVE KEYBOARD & MOUSE EVENT WINDOW PROCESSING TRACKERS
// =========================================================================

void AppController::KeyboardCallback(unsigned char key, int x, int y) {
    if (!s_instance) return;

    const double zoomSensitivity = 0.1;

    switch (key) {
     case '+':
    {
        // Zoom in trajectory depends on target and position
        Eigen::Vector3d viewDir = (s_instance->m_scene.camTarget - s_instance->m_scene.camPos).normalized();
        s_instance->m_scene.camPos += viewDir * zoomSensitivity;
        break;
    }

    case '-':
    {
        // Zoom out trajectory depends on target and position
        Eigen::Vector3d viewDir = (s_instance->m_scene.camTarget - s_instance->m_scene.camPos).normalized();
        s_instance->m_scene.camPos -= viewDir * zoomSensitivity;
        break;
    }

    case 'p':
    case 'P':
        // flat or perpective rendering not available for Mesh/MC rendering
        if (s_instance->currentMode != AppController::SceneMode::CADjunction) {
            s_instance->m_is3D = !s_instance->m_is3D;
            std::cout << "[RENDER] Switched to " << (s_instance->m_is3D ? "3D Perspective" : "2D Flat") << std::endl;
            // CRITICAL: Force grid re-compilation because spatial coordinates heights are statically compiled into display lists
            s_instance->ComputeAndBufferMesh(s_instance->m_sampling);
        }
        break;

    case 'i':
    case 'I':
        if (s_instance->currentMode != AppController::SceneMode::CADjunction) {
            s_instance->m_isIntersection = !s_instance->m_isIntersection;
            std::cout << "\n[BOOL Operation] switched to: " << (s_instance->m_isIntersection ? "INTERSECTION\n" : "UNION\n") << std::endl;
            s_instance->updateActiveListIndex();
        }
        break;

    case 'r':
    case 'R':
        // Reset viewport tracking orientation tables
        s_instance->m_scene.Object_Move = Eigen::Vector3d(0.0, 0.0, 0.0);
        s_instance->m_scene.rotation_matrix = Eigen::Matrix4d::Identity();
        std::cout << "[VIEW] Resetting rotation and zoom" << std::endl;
        break;

    case '0': case '1': case '2': case '3': case '4': case '5': case '6': case '7':
    {
        // various rendering are not available for Mesh/MC rendering
        if (s_instance->currentMode != AppController::SceneMode::CADjunction) {

            // ASCII translation pattern extracting targeted index updates
            if (key - '0' != s_instance->m_activeMethod)
            {
                s_instance->m_activeMethod = key - '0';
                s_instance->updateActiveListIndex();
            }
        }
    } break;

    case 'w': case 'W':
    {
        glDisable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); // Enable wireframe debug view
    } break;

    case 'f': case 'F':
    {
        glEnable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); // Restore solid mesh shading
    } break;

    case 27: // System close call triggered via Escape key mapping bounds
        std::cout << "[SYSTEM] Closing application..." << std::endl;
        exit(0);
        break;

    default:
        break;
    }

    glutPostRedisplay();
}

/**
 * @brief Principal display callback routine driving the main OpenGL rendering loop.
 * @details Orthogonally isolates camera layouts, object space trackball matrices,
 * and 2D HUD widget overlays. Synchronizes fixed-pipeline illumination vectors
 * conditionally based on the active scene mode to guarantee exact color mapping
 * or stable specular shading.
 */
void AppController::DisplayCallback() {
    if (!s_instance) return;

    // 1. Initialize clean frame buffers and depth testing context
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    // 2. Establish global Viewport Observation matrix transformation via dynamic camera coordinates
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(
        s_instance->m_scene.camPos.x(), s_instance->m_scene.camPos.y(), s_instance->m_scene.camPos.z(),
        s_instance->m_scene.camTarget.x(), s_instance->m_scene.camTarget.y(), s_instance->m_scene.camTarget.z(),
        0.0, 1.0, 0.0
    );

    // =========================================================================
    // HARDWARE LIGHTING CONFIGURATION & TRANSFORMATION BINDING
    // =========================================================================
    if (s_instance->currentMode == SceneMode::CADjunction) {
        //GLfloat light_position[] = { 1.0f, 1.0f, 10.0f, 0.0f }; // Infinite directional sun vector
        //glLightfv(GL_LIGHT0, GL_POSITION, light_position);
        glEnable(GL_LIGHTING);
        //glEnable(GL_LIGHT0);
    }
    else {
        glDisable(GL_LIGHTING);
    }

    // 3. Render Solid Object Space Geometries
    glPushMatrix();

    // Step 1: Apply screen-aligned panning translations (Global X and Y screen-space shifts)
    glTranslated(s_instance->m_scene.Object_Move[0],
        s_instance->m_scene.Object_Move[1],
        0.0); // Z translation is safely set to 0.0 since zoom is handled via camPos

    // Step 2: Apply cumulative virtual trackball rotations around the coordinate origin
    glMultMatrixd(s_instance->m_scene.rotation_matrix.data());

    // Call selected compiled logic display lists
    int m = s_instance->m_activeMethod;
    int op = s_instance->m_isIntersection ? 0 : 1;

    if (m >= 0 && m < (int)s_instance->m_scene.RF_ogl_lists.size()) {
        int id_to_draw = s_instance->m_scene.RF_ogl_lists[m][op];
        if (glIsList(id_to_draw)) {
            glCallList(id_to_draw);
        }
    }

    // 4. Overlay Underlying Primitive Boundary Contours (Bypass depth and shading blocks)
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING); // Forces emissive white lines unaffected by lighting states
    glLineWidth(2.5f);
    glColor3d(1.0, 1.0, 1.0);

    for (const auto& obj : s_instance->m_objects) {
        if (auto sphere = std::dynamic_pointer_cast<ImplicitSphere>(obj)) {
            s_instance->m_scene.DrawSphereContour(
                sphere->getCenter(),
                sphere->getRadius(),
                Eigen::Vector3d(1.0, 1.0, 1.0)
            );
        }
    }

    glLineWidth(1.0f);
    glEnable(GL_DEPTH_TEST);
    glPopMatrix(); // End of Object Space transformations block

    // =========================================================================
    // 2D HUD MATRIX OVERLAY: SPATIAL ORIENTATION WIDGET
    // =========================================================================
    int width = glutGet(GLUT_WINDOW_WIDTH);
    int height = glutGet(GLUT_WINDOW_HEIGHT);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, width, 0, height, -100, 100);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    // Clear depth values locally to guarantee the HUD sits seamlessly on top of 3D geometry layers
    glClear(GL_DEPTH_BUFFER_BIT);

    glTranslated(60, 60, 0);
    glMultMatrixd(s_instance->m_scene.rotation_matrix.data());

    glDisable(GL_LIGHTING); // Keep widget indicators purely emissive
    glLineWidth(2.5f);
    glBegin(GL_LINES);
    glColor3d(1.0, 0.0, 0.0); glVertex3d(0, 0, 0); glVertex3d(35, 0, 0);  // X Axis (Red)
    glColor3d(0.0, 0.8, 0.0); glVertex3d(0, 0, 0); glVertex3d(0, 35, 0);  // Y Axis (Green)
    glColor3d(0.0, 0.0, 1.0); glVertex3d(0, 0, 0); glVertex3d(0, 0, 35);  // Z Axis (Blue)
    glEnd();
    glLineWidth(1.0f);

    // Restore projection matrices stacks to their original standard state configurations
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);

    glutSwapBuffers();
}

void AppController::ReshapeCallback(int w, int h) {
    if (h == 0) h = 1;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (double)w / h, 0.1, 100.0);
    glMatrixMode(GL_MODELVIEW);
}

void AppController::MouseCallback(int button, int state, int x, int y) {
    if (!s_instance) return;
    if (button == GLUT_LEFT_BUTTON) {
        // Track mouse down events to drive interactive trackball rotation steps
        s_instance->m_scene.is_rotating = (state == GLUT_DOWN);
        s_instance->m_scene.previous_mouse_position = Eigen::Vector2i(x, y);
    }
}

void AppController::MotionCallback(int x, int y) {
    if (!s_instance || !s_instance->m_scene.is_rotating) return;

    double dx = (x - s_instance->m_scene.previous_mouse_position[0]) * 0.5;
    double dy = (y - s_instance->m_scene.previous_mouse_position[1]) * 0.5;

    // Assembles incremental Affine transformation step matrices tracking drag displacements
    Eigen::Affine3d rot = Eigen::Affine3d(
        Eigen::AngleAxisd(dx * 0.01, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(dy * 0.01, Eigen::Vector3d::UnitX())
    );

    s_instance->m_scene.rotation_matrix = rot.matrix() * s_instance->m_scene.rotation_matrix;
    s_instance->m_scene.previous_mouse_position = Eigen::Vector2i(x, y);

    glutPostRedisplay();
}

// =========================================================================
// GRADIENT FIELD BENCHMARKS & QUALITY METRICS ANALYSIS
// =========================================================================

void AppController::BenchmarkGradient(
    bool isInter,
    int N,
    double field_value_start,
    double field_value_end,
    int number_of_runs)
{
    std::string filename = "BenchmarkGradient.csv";
    std::ofstream csvFile(filename);

    if (!csvFile.is_open()) {
        std::cerr << "Error: Could not create " << filename << ". Close Excel!" << std::endl;
        return;
    }

    csvFile << "Offset,Function,Mean,StdDev,Error" << std::endl;

    double gap = (field_value_end - field_value_start) / (double)number_of_runs;

    // Incremental step simulation passes testing gradient properties across shifting values bands
    for (int i = 0; i <= number_of_runs; ++i)
    {
        double field_offset = field_value_start + i * gap;
        std::cout << " Gradient Benchmark --- Field offset = " << field_offset << std::endl;

        // Reset buffers to isolate testing steps observations structures
        EvaluatedPoints.clear();
        G_RvachevNd.clear(); G_ZenkinNd.clear();
        G_Sph_Directional.clear(); G_Sph_Normalized.clear();
        F_RvachevNd.clear(); F_ZenkinNd.clear();
        F_Sph_Directional.clear(); F_Sph_Normalized.clear();

        m_objects.clear();
        m_objects.push_back(std::make_shared<ImplicitSphere>(Eigen::Vector3d(-0.5, -0.5, 0.0), 1.0));
        m_objects.push_back(std::make_shared<ImplicitSphere>(Eigen::Vector3d(0.5, -0.5, 0.0), 0.8));
        m_objects.push_back(std::make_shared<ImplicitSphere>(Eigen::Vector3d(0.0, 0.5, 0.0), 1.2));

        if (m_objects.size() < 3) return;

        // Step A: Discrete spatial configurations allocation passes generating circular rings per primitive
        std::vector<std::vector<Eigen::Vector3d>> CircularPoints;
        for (const auto& obj : m_objects) {
            auto sphere = std::dynamic_pointer_cast<ImplicitSphere>(obj);
            if (sphere) {
                Eigen::Vector3d C = sphere->getCenter();
                double R = sphere->getRadius();
                double r_sq = R * R - C.z() * C.z();
                if (r_sq < 0) continue;

                double r_z0 = std::sqrt(r_sq) - field_offset;
                if (r_z0 < 0) continue;

                std::vector<Eigen::Vector3d> points;
                points.reserve(N);
                for (int j = 0; j < N; ++j) {
                    double theta = (2.0 * M_PI * j) / N;
                    points.emplace_back(C.x() + r_z0 * std::cos(theta), C.y() + r_z0 * std::sin(theta), 0.0);
                }
                CircularPoints.push_back(std::move(points));
            }
        }

        // Step B: Filtration passes: retain points matching composite switching boundary conditions
        for (size_t idx = 0; idx < CircularPoints.size(); ++idx) {
            for (const auto& P : CircularPoints[idx]) {
                double v0 = m_objects[0]->Evaluate(P);
                double v1 = m_objects[1]->Evaluate(P);
                double v2 = m_objects[2]->Evaluate(P);
                double f_val = isInter ? std::min({ v0, v1, v2 }) : std::max({ v0, v1, v2 });
                double v_current = m_objects[idx]->Evaluate(P);

                if (std::abs(f_val - v_current) < 1e-3 && std::abs(f_val - field_offset) < 1e-3) {
                    EvaluatedPoints.push_back(P);
                }
            }
        }

        RCore::SphericalEvaluator F;
        F.Init(3);
        int m_param = 0;
        double h = 1e-4;

        // Step C: Numerical central finite differences schemes computing gradient spatial elements
        for (const auto& P : EvaluatedPoints) {
            Eigen::VectorXd Pxd(3);
            for (int j = 0; j < 3; ++j) Pxd[j] = m_objects[j]->Evaluate(P);

            auto EvalZenkin = [&](const Eigen::Vector3d& pt) {
                Eigen::VectorXd X(3);
                for (int j = 0; j < 3; ++j) X[j] = m_objects[j]->Evaluate(pt);
                return RCore::ZenkinNd(X, isInter);
                };

            auto EvalRvachev = [&](const Eigen::Vector3d& pt) {
                Eigen::VectorXd X(3);
                for (int j = 0; j < 3; ++j) X[j] = m_objects[j]->Evaluate(pt);
                return RCore::RvachevNd(X, isInter, m_param);
                };

            auto EvalSphDir = [&](const Eigen::Vector3d& pt) {
                Eigen::VectorXd X(3);
                for (int j = 0; j < 3; ++j) X[j] = m_objects[j]->Evaluate(pt);
                return F.Compute(isInter, X);
                };

            auto EvalSphNorm = [&](const Eigen::Vector3d& pt) {
                Eigen::VectorXd X(3);
                for (int j = 0; j < 3; ++j) X[j] = m_objects[j]->Evaluate(pt);
                return F.ComputeNormalized(isInter, X);
                };

            auto ComputeSpatialGrad = [&](auto& func) {
                double gx = (func(P + Eigen::Vector3d(h, 0, 0)) - func(P - Eigen::Vector3d(h, 0, 0))) / (2.0 * h);
                double gy = (func(P + Eigen::Vector3d(0, h, 0)) - func(P - Eigen::Vector3d(0, h, 0))) / (2.0 * h);
                double gz = (func(P + Eigen::Vector3d(0, 0, h)) - func(P - Eigen::Vector3d(0, 0, h))) / (2.0 * h);
                return Eigen::Vector3d(gx, gy, gz);
                };

            G_ZenkinNd.push_back(ComputeSpatialGrad(EvalZenkin));
            G_RvachevNd.push_back(ComputeSpatialGrad(EvalRvachev));
            G_Sph_Directional.push_back(ComputeSpatialGrad(EvalSphDir));
            G_Sph_Normalized.push_back(ComputeSpatialGrad(EvalSphNorm));
        }

        // Step D: Statistical logging tracking variance shifts and deviation from unit magnitude (|grad| -> 1)
        auto LogStats = [&](const auto& gradients, const std::string& label) {
            if (gradients.empty()) return;
            double sum_mag = 0.0, sum_sq_mag = 0.0;
            for (const auto& g : gradients) {
                double mag = g.norm();
                sum_mag += mag; sum_sq_mag += mag * mag;
            }
            double mean = sum_mag / gradients.size();
            double var = (sum_sq_mag / gradients.size()) - (mean * mean);
            double std_dev = std::sqrt(std::max(0.0, var));

            csvFile << std::fixed << std::setprecision(6)
                << field_offset << "," << label << "," << mean << ","
                << std_dev << "," << std::abs(mean - 1.0) << std::endl;
            };

        LogStats(G_ZenkinNd, "ZENKIN");
        LogStats(G_RvachevNd, "RVACHEV");
        LogStats(G_Sph_Directional, "SPH_DIR");
        LogStats(G_Sph_Normalized, "SPH_NORM");

        std::cout << "Done: " << EvaluatedPoints.size() << " pts." << std::endl;
    }
    csvFile.close();
}


// benchmark for Gradient accuracy and stability for a 9 primitives combination 
// evaluate various Rfunctions (Ro, Rp, Rm, Chained Rvachev functions, Zenkins, Ternary operator, Normalized ternary operator)
// Primitive number = 9 is fixed.

// further work : update the code for general number of primitives number = 3^k...

void AppController::BenchmarkGradientUnbalancedTree(
    bool isInter,
    int N,
    double field_value_start,
    double field_value_end,
    int number_of_runs)
{
    std::string filename = "BenchmarkGradientUnbalanced.csv";
    std::ofstream csvFile(filename);
    if (!csvFile.is_open()) return;

    csvFile << "Offset,Function,Mean,StdDev,Error" << std::endl;


    const int SphereNumbers = 9;

    std::vector<std::shared_ptr<ImplicitObject> > daisy = MakeDaisy(1.0,0.5,SphereNumbers);
    m_objects.clear();
    m_objects = daisy;

    double gap = (field_value_end - field_value_start) / (double)number_of_runs;
    double h = 1e-4;

    for (int step = 0; step <= number_of_runs; ++step) {
        double field_offset = field_value_start + step * gap;
        std::cout << ">>> Processing Offset: " << field_offset << " (" << step << "/" << number_of_runs << ")" << std::endl;

        cleanAlldata();

        // Sample and filter spatial testing coordinates along the multi-sphere daisy petals
        for (size_t s = 0; s < daisy.size(); ++s) {
            auto sphere = std::dynamic_pointer_cast<ImplicitSphere>(daisy[s]);
            double r_z0 = sphere->getRadius() - field_offset;
            if (r_z0 < 0) continue;

            for (int j = 0; j < N; ++j) {
                double theta = (2.0 * M_PI * j) / N;
                Eigen::Vector3d P(sphere->getCenter().x() + r_z0 * std::cos(theta),
                    sphere->getCenter().y() + r_z0 * std::sin(theta), 0.0);

                double f_ref = daisy[0]->Evaluate(P);
                for (int k = 1; k < 9; ++k) {
                    double v = daisy[k]->Evaluate(P);
                    f_ref = isInter ? std::min(f_ref, v) : std::max(f_ref, v);
                }

                if (std::abs(f_ref - daisy[s]->Evaluate(P)) < 1e-3 && std::abs(f_ref - field_offset) < 1e-3) {
                    EvaluatedPoints.push_back(P);
                }
            }
        }

        // Evaluate gradient structures across deep binary cascades versus proposed balanced trees
        for (const auto& P : EvaluatedPoints) {

            // Topology A: Unbalanced evaluation configurations (deep comb layouts)
            auto eval_rp2 = [&](const Eigen::Vector3d& pt) {
                double res = daisy[0]->Evaluate(pt);
                for (size_t i = 1; i < 9; ++i) res = RCore::Rp(res, daisy[i]->Evaluate(pt), isInter);
                return res;
                };

            auto eval_r0m1 = [&](const Eigen::Vector3d& pt) {
                double res = daisy[0]->Evaluate(pt);
                for (size_t i = 1; i < 9; ++i) res = RCore::R0m(res, daisy[i]->Evaluate(pt), isInter, 1);
                return res;
                };

            // Topology B: Direct single-layer flat N-ary formulations
            auto eval_rvachev_n_direct = [&](const Eigen::Vector3d& pt) {
                Eigen::VectorXd X(9);
                for (int i = 0; i < 9; ++i) X(i) = daisy[i]->Evaluate(pt);
                return RCore::RvachevNd(X, isInter, 1);
                };

            auto eval_zenkin_n_direct = [&](const Eigen::Vector3d& pt) {
                Eigen::VectorXd X(9);
                for (int i = 0; i < 9; ++i) X(i) = daisy[i]->Evaluate(pt);
                return RCore::ZenkinNd(X, isInter);
                };

            // Topology C: Proposed structured balanced ternary tree architecture models
            auto eval_sph_dir_tree = [&](const Eigen::Vector3d& pt) {
                Eigen::VectorXd level1(3);
                for (int i = 0; i < 3; ++i) {
                    Eigen::VectorXd sub(3);
                    sub << daisy[i * 3]->Evaluate(pt), daisy[i * 3 + 1]->Evaluate(pt), daisy[i * 3 + 2]->Evaluate(pt);
                    level1(i) = SpherEval.Compute(isInter, sub);
                }
                return SpherEval.Compute(isInter, level1);
                };

            auto eval_sph_norm_tree = [&](const Eigen::Vector3d& pt) {
                Eigen::VectorXd level1(3);
                for (int i = 0; i < 3; ++i) {
                    Eigen::VectorXd sub(3);
                    sub << daisy[i * 3]->Evaluate(pt),
                        daisy[i * 3 + 1]->Evaluate(pt),
                        daisy[i * 3 + 2]->Evaluate(pt);
                    level1(i) = SpherEval.ComputeNormalized(isInter, sub);
                }
                return SpherEval.ComputeNormalized(isInter, level1);
                };

            auto grad_fd = [&](auto& func) {
                double gx = (func(P + Eigen::Vector3d(h, 0, 0)) - func(P - Eigen::Vector3d(h, 0, 0))) / (2.0 * h);
                double gy = (func(P + Eigen::Vector3d(0, h, 0)) - func(P - Eigen::Vector3d(0, h, 0))) / (2.0 * h);
                double gz = (func(P + Eigen::Vector3d(0, 0, h)) - func(P - Eigen::Vector3d(0, 0, h))) / (2.0 * h);
                return Eigen::VectorXd(Eigen::Vector3d(gx, gy, gz));
                };

            G_RpBinaryTree.push_back(grad_fd(eval_rp2));
            G_R0mBinaryTree.push_back(grad_fd(eval_r0m1));
            G_RvachevNd.push_back(grad_fd(eval_rvachev_n_direct));
            G_ZenkinNd.push_back(grad_fd(eval_zenkin_n_direct));
            G_Sph_Directional.push_back(grad_fd(eval_sph_dir_tree));
            G_Sph_Normalized.push_back(grad_fd(eval_sph_norm_tree));
        }

        auto Log = [&](const std::vector<Eigen::VectorXd>& grads, const std::string& label) {
            if (grads.empty()) return;
            double sum = 0, sum_sq = 0;
            for (const auto& g : grads) { double m = g.norm(); sum += m; sum_sq += m * m; }
            double mean = sum / (double)grads.size();
            double var = (sum_sq / (double)grads.size()) - (mean * mean);
            csvFile << field_offset << "," << label << "," << mean << "," << std::sqrt(std::max(0.0, var)) << "," << std::abs(mean - 1.0) << std::endl;
            };

        Log(G_RpBinaryTree, "Rp_BIN_COMB");
        Log(G_R0mBinaryTree, "R0m1_BIN_COMB");
        Log(G_RvachevNd, "RVACHEV_9-ARY_DIRECT");
        Log(G_ZenkinNd, "ZENKIN_9-ARY_DIRECT");
        Log(G_Sph_Directional, "SPH_DIR_TREE");
        Log(G_Sph_Normalized, "SPH_NORM_TREE");
    }
    csvFile.close();
    std::cout << ">>> Unbalanced Tree Benchmark Complete. CSV generated." << std::endl;
}

void AppController::RunNewtonRaphsonBenchmark(int gridRes) {
    const double epsilon = 1e-6;
    const int maxIter = 2000;
    const double h = 1e-6;
    const double sutureThreshold = 1e-6; // Bounds non-differentiable switching ridges lines (sutures)




    std::string filename = "BenchmarkNewtonConvergence.csv";
    std::ofstream outFile(filename);

    if (!outFile.is_open()) {
        std::cerr << "Error: Could not open " << filename << " for writing." << std::endl;
        return;
    }

    outFile << "Method,SuccessRate,AvgIterations,AvgResidual,FailureRate_Gradient,FailureRate_MaxIter,SuturePointsRate" << std::endl;

    Eigen::Vector3d pMin(-2, -2, -2);
    Eigen::Vector3d pMax(2, 2, 2);

    Eigen::Vector3d step;
    if (gridRes > 1) {
        step = (pMax - pMin) / static_cast<double>(gridRes - 1);
    }
    else {
        step = Eigen::Vector3d::Zero();
    }

    //initialise 3 Spheres as in paper to run benchmark

    makeSpheres();

    auto get_X = [](const Eigen::Vector3d& p) {
        Eigen::VectorXd X(3);
        for (int i = 0; i < 3; ++i) {
            X(i) = s_instance->m_objects[i]->Evaluate(p);
        }
        return X;
        };

    struct TestMethod {
        std::string name;
        std::function<double(const Eigen::Vector3d&)> eval;
    };

    std::vector<TestMethod> methods = {
        {"Rp_BIN_COMB", [get_X](const Eigen::Vector3d& p) {
            Eigen::VectorXd X = get_X(p);
            return RCore::Rp(RCore::Rp(X(0), X(1), true), X(2), true);
        }},
        {"R0m1_BIN_COMB", [get_X](const Eigen::Vector3d& p) {
            Eigen::VectorXd X = get_X(p);
            return RCore::R0m(RCore::R0m(X(0), X(1), true, 1), X(2), true, 1);
        }},
        {"RVACHEV_3-ARY_DIRECT", [get_X](const Eigen::Vector3d& p) {
            return RCore::RvachevNd(get_X(p), true, 1);
        }},
        {"ZENKIN_3-ARY_DIRECT", [get_X](const Eigen::Vector3d& p) {
            return RCore::ZenkinNd(get_X(p), true);
        }},
        {"SPH_DIR_TERNARY", [get_X](const Eigen::Vector3d& p) {
            return s_instance->SpherEval.Compute(true, get_X(p));
        }},
        {"SPH_NORM_TERNARY", [get_X](const Eigen::Vector3d& p) {
            return s_instance->SpherEval.ComputeNormalized(true, get_X(p));
        }}
    };

    for (auto& method : methods) {
        std::cout << "Testing: " << method.name << " (" << gridRes << "^3 points)..." << std::endl;

        long long totalPoints = 0;
        int successfulPoints = 0;
        int failGradient = 0;
        int failMaxIter = 0;
        int sutureTouchCount = 0; // Tracks optimization lines encountering non-differentiable singularities
        long long totalIterations = 0;
        double totalResidual = 0.0;

        // Thread-safe parallelized projection tracing across the entire grid cube utilizing OpenMP
#pragma omp parallel for reduction(+:successfulPoints, failGradient, failMaxIter, totalIterations, totalResidual, totalPoints, sutureTouchCount)
        for (int i = 0; i < gridRes; ++i) {
            for (int j = 0; j < gridRes; ++j) {
                for (int k = 0; k < gridRes; ++k) {

                    //std::cout << "boucle " << i << " " << j << " " << k << "\n";
                    Eigen::Vector3d p(pMin.x() + i * step.x(),
                        pMin.y() + j * step.y(),
                        pMin.z() + k * step.z());

                    int it = 0;
                    bool converged = false;
                    bool gradientDead = false;
                    bool hitSuture = false;

                    while (it < maxIter) {
                        Eigen::VectorXd X = get_X(p);

                        // Empirical suture detection criteria checking equality boundaries x_i = x_j
                        if (std::abs(X(0) - X(1)) < sutureThreshold ||
                            std::abs(X(1) - X(2)) < sutureThreshold ||
                            std::abs(X(0) - X(2)) < sutureThreshold) {
                            hitSuture = true;
                        }

                        double val = method.eval(p);
                        if (std::abs(val) < epsilon) {
                            converged = true;
                            break;
                        }

                        //std::cout << "ok1\n";
                        // Local spatial gradient extraction via central finite differences loops
                        Eigen::Vector3d grad;
                        grad.x() = (method.eval(p + Eigen::Vector3d(h, 0, 0)) - method.eval(p - Eigen::Vector3d(h, 0, 0))) / (2.0 * h);
                        grad.y() = (method.eval(p + Eigen::Vector3d(0, h, 0)) - method.eval(p - Eigen::Vector3d(0, h, 0))) / (2.0 * h);
                        grad.z() = (method.eval(p + Eigen::Vector3d(0, 0, h)) - method.eval(p - Eigen::Vector3d(0, 0, h))) / (2.0 * h);

                        //std::cout << "ok2\n";
                        double g2 = grad.squaredNorm();
                        if (g2 < 1e-14) {
                            gradientDead = true; // Optimization path trapped inside critical flat potential zones
                            break;
                        }

                        // Project positions along normal directions using standard multi-dimensional Newton steps
                        p = p - (val / g2) * grad;
                        it++;
                    }

                    totalPoints++;
                    if (hitSuture) sutureTouchCount++;

                    if (converged) {
                        successfulPoints++;
                        totalIterations += it;
                        totalResidual += std::abs(method.eval(p));
                    }
                    else if (gradientDead) {
                        failGradient++;
                    }
                    else {
                        failMaxIter++;
                    }
                }
            }
        }

        double successRate = (totalPoints > 0) ? (double)successfulPoints / totalPoints * 100.0 : 0;
        double fGradRate = (totalPoints > 0) ? (double)failGradient / totalPoints * 100.0 : 0;
        double fMaxRate = (totalPoints > 0) ? (double)failMaxIter / totalPoints * 100.0 : 0;
        double sutureRate = (totalPoints > 0) ? (double)sutureTouchCount / totalPoints * 100.0 : 0;
        double avgIt = (successfulPoints > 0) ? (double)totalIterations / successfulPoints : 0;
        double avgRes = (successfulPoints > 0) ? totalResidual / successfulPoints : 0;

        outFile << method.name << ","
            << successRate << ","
            << avgIt << ","
            << avgRes << ","
            << fGradRate << ","
            << fMaxRate << ","
            << sutureRate << std::endl;
    }

    outFile.close();
    std::cout << "Benchmark complete. Results saved in " << filename << std::endl;
}

// =========================================================================
// SEQUENTIAL CHRONO-ISOLATED PIPELINE BENCHMARK CORE
// =========================================================================

void AppController::RunFullBenchmark() {
    std::vector<LogicType> logics = { LogicType::MIN_NAIVE, LogicType::BINARY_RP, LogicType::TERNARY_KRF };
    std::vector<int> resolutions = { 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150 };
    std::vector<BenchResult> allResults;

    Eigen::Vector3d minBound(-1.5, -1.5, -1.5);
    Eigen::Vector3d maxBound(1.5, 1.5, 1.5);

    std::cout << "\n-------------------\n>>> Starting Full Comparative Pipeline Benchmark (Chrono-Isolated)..." << std::endl;

    for (int res : resolutions) {
        double dx = (maxBound.x() - minBound.x()) / (double)res;
        double initEps = dx * 0.01;
        double finalEps = initEps * 2.0;
        double finalAreaThr = finalEps * finalEps;
        double sharpAngle = 20.0;

        for (LogicType logic : logics) {
            // Reset local buffers and parameter bindings before starting calculations
            Mesh.ClearAll();
            m_scene.m_CadLogic->currentLogic = logic;
            Mesh.TernarySphericalOperator = &SpherEval;
            m_scene.m_CadLogic->TernarySphericalOperator = &SpherEval;

            std::string logicLabel = (logic == LogicType::MIN_NAIVE ? "Min" : (logic == LogicType::BINARY_RP ? "Rp_Sym" : "Ternary"));
            std::cout << "\n----------\n[BENCH] Processing Logic [" << logicLabel << "] @ Res " << res << "..." << std::endl;

            auto totalDuration = std::chrono::nanoseconds(0);

            // -----------------------------------------------------------------
            // PHASE 1: GENERATION, WELDING & CLEANUP (CHRONO INCLUDED)
            // -----------------------------------------------------------------
            auto tStart1 = std::chrono::high_resolution_clock::now();

            RunMarchingCube(minBound, maxBound, res);
            Mesh.WeldVertices(initEps);
            RunTopologyCleanup(initEps);
            Mesh.buildMeshTopology(); // Recompile structural connectivity references lists

            auto tEnd1 = std::chrono::high_resolution_clock::now();
            totalDuration += std::chrono::duration_cast<std::chrono::nanoseconds>(tEnd1 - tStart1);

            // -----------------------------------------------------------------
            // PHASE 2: TOPOLOGICAL OPTIMIZATION (CHRONO EXCLUDED)
            // -----------------------------------------------------------------
            // Excluded from chrono since optimization operations are purely simplicial (no field queries)
            RunValenceOptimization();
            RunParasiteCollapse(initEps);
            Mesh.buildMeshTopology();

            // -----------------------------------------------------------------
            // PHASES 3 TO 5: GEOMETRIC RELAXATION & SANITIZATION (CHRONO INCLUDED)
            // -----------------------------------------------------------------
            auto tStart2 = std::chrono::high_resolution_clock::now();

            // Phase 3: Drive safe tangential laplacian smoothing tracking steps followed by Newton updates
            RunTriangleImprovement();
            AdjustPoints(*m_scene.m_CadLogic, Mesh.vertices);
            Mesh.buildMeshTopology();

            // Phase 4: Ridge sub-sampling tracing high-curvature lines structures
            m_scene.m_MCGenerator->RefineSharpEdges(*m_scene.m_CadLogic, Mesh, sharpAngle);

            // Phase 5: Multi-criteria structural polishing filters passes
            RunSmallTriangleCollapse(finalAreaThr);
            RunUnifiedCleanup(finalEps);
            RunFeaturePreservingSmooth(sharpAngle, 10);
            RunUltimateMeshSanitizer(finalAreaThr * 2, 0.06);

            Mesh.buildMeshTopology(); // Final synchronization pass securing normals integrity

            auto tEnd2 = std::chrono::high_resolution_clock::now();
            totalDuration += std::chrono::duration_cast<std::chrono::nanoseconds>(tEnd2 - tStart2);

            // -----------------------------------------------------------------
            // METRICS EVALUATION & CSV METRICS STORAGE LOGS
            // -----------------------------------------------------------------
            double duration_ms = totalDuration.count() / 1e6;
            double sumErr = 0, maxErr = 0;

            for (const auto& v : Mesh.vertices) {
                double err = std::abs(m_scene.m_CadLogic->Evaluate(v));
                sumErr += err;
                if (err > maxErr) maxErr = err;
            }

            BenchResult r;
            r.logicName = logicLabel;
            r.resolution = res;
            r.totalTime_ms = duration_ms;
            r.finalFaceCount = (int)Mesh.faces.size();
            r.meanError = sumErr / (double)Mesh.vertices.size();
            r.maxError = maxErr;
            allResults.push_back(r);

            std::cout << "\t-> Simulation finished in " << duration_ms << " ms. Triangles: " << r.finalFaceCount << std::endl;
            std::cout << "----------\n";
        }
    }
    SaveResultsToCSV(allResults); // Export metrics structures tables onto spreadsheet files
}

void AppController::SaveResultsToCSV(const std::vector<BenchResult>& results) {
    std::ofstream file("benchmark_final.csv");

    if (!file.is_open()) {
        std::cerr << "[ERROR] Cannot create the CSV file." << std::endl;
        return;
    }

    file << "Logic;Resolution;TotalTime_ms;FaceCount;MeanError;MaxError\n";

    for (const auto& r : results) {
        file << std::defaultfloat << r.logicName << ";" << r.resolution << ";";
        file << std::fixed << std::setprecision(2) << r.totalTime_ms << ";";
        file << std::defaultfloat << r.finalFaceCount << ";";
        file << std::scientific << std::setprecision(8) << r.meanError << ";" << r.maxError << "\n";
    }
    file.close();
    std::cout << "\n[SUCCESS] CSV genere avec succes (incluant Mean et Max Error)." << std::endl;
}

// =========================================================================
// PIPELINE CAPSULES & IN-PLACE SIMPLICIAL OPTIMIZERS
// =========================================================================

void AppController::RunMarchingCube(const Eigen::Vector3d& minBound, const Eigen::Vector3d& maxBound, int gridResolution) {
    if (!m_scene.m_MCGenerator || !m_scene.m_CadLogic) {
        throw std::runtime_error("MCGenerator or CadLogic not initialized.");
    }

    // Call implicit Marching Cubes surface mesh structural extraction algorithms
    auto mcResult = m_scene.m_MCGenerator->Generate(*(m_scene.m_CadLogic), minBound, maxBound, gridResolution);

    // Pass outputs directly into member buffers utilizing performance move operations semantics
    Mesh.vertices = std::move(mcResult.first);
    Mesh.faces = std::move(mcResult.second);
}

void AppController::RunTopologyCleanup(double epsilon) {
    if (!m_scene.m_MCGenerator) {
        throw std::runtime_error("MCGenerator not initialized for topology cleanup.");
    }

    // Erase degenerate spatial cell patches following Marching Cubes initialization errors
    auto passInit = m_scene.m_MCGenerator->UnifiedClean(Mesh.vertices, Mesh.faces, epsilon);

    Mesh.vertices = std::move(passInit.first);
    Mesh.faces = std::move(passInit.second);
}

int AppController::RunValenceOptimization(int maxPasses) {
    if (!m_scene.m_MCGenerator || !m_scene.m_CadLogic) {
        throw std::runtime_error("MCGenerator or CadLogic not initialized for valence optimization.");
    }

    int totalFlips = 0;
    // Sequential pass blocks performing localized edge flips optimizing node degrees distribution toward valence 6
    for (int pass = 0; pass < maxPasses; ++pass) {
        int flips = m_scene.m_MCGenerator->OptimizeValenceFlips(*(m_scene.m_CadLogic), Mesh.vertices, Mesh.faces);
        totalFlips += flips;
        if (flips == 0) break; // Break processing once local structural transitions reach convergence
    }
    return totalFlips;
}

void AppController::RunParasiteCollapse(double epsilonDistance) {
    // Instantiates a local neighborhood tracker checking connectivity properties
    NeighborMesh mesh;
    mesh.vertices = this->Mesh.vertices;
    mesh.faces = this->Mesh.faces;
    mesh.Build_P2P_Neigh();
    mesh.Build_P2F_Neigh();

    std::vector<bool> faceRemoved(this->Mesh.faces.size(), false);
    std::vector<bool> vertexModified(this->Mesh.vertices.size(), false);

    std::vector<Eigen::Vector3i> cleanFaces;
    cleanFaces.reserve(this->Mesh.faces.size());

    int collapseCount = 0;

    // Detect and collapse artificial flat valence-3 pyramids generated on sharp implicit junction lines
    for (size_t vIdx = 0; vIdx < this->Mesh.vertices.size(); ++vIdx)
    {
        // Valence 3 criteria checks: check if vertex is shared by exactly 3 edges and 3 faces
        if (mesh.P2P_Neigh[vIdx].size() != 3 || mesh.P2F_Neigh[vIdx].size() != 3) continue;

        const auto& adjFaces = mesh.P2F_Neigh[vIdx];

        bool locked = vertexModified[vIdx];
        for (size_t fIdx : adjFaces) {
            if (faceRemoved[fIdx]) { locked = true; break; }
        }
        if (locked) continue;

        // Trace base vertices elements (A, B, C) matching structural face winding tracking lists
        size_t refFaceIdx = *adjFaces.begin();
        Eigen::Vector3i refF = this->Mesh.faces[refFaceIdx];

        size_t A = 0, B = 0, C = 0;
        if ((size_t)refF.x() == vIdx) { A = refF.y(); B = refF.z(); }
        else if ((size_t)refF.y() == vIdx) { A = refF.z(); B = refF.x(); }
        else { A = refF.x(); B = refF.y(); }

        for (size_t fIdx : adjFaces) {
            if (fIdx == refFaceIdx) continue;
            Eigen::Vector3i F = this->Mesh.faces[fIdx];
            if ((size_t)F.x() != vIdx && (size_t)F.x() != A && (size_t)F.x() != B) { C = F.x(); break; }
            if ((size_t)F.y() != vIdx && (size_t)F.y() != A && (size_t)F.y() != B) { C = F.y(); break; }
            if ((size_t)F.z() != vIdx && (size_t)F.z() != A && (size_t)F.z() != B) { C = F.z(); break; }
        }

        if (vertexModified[A] || vertexModified[B] || vertexModified[C]) continue;

        Eigen::Vector3d pA = this->Mesh.vertices[A], pB = this->Mesh.vertices[B], pC = this->Mesh.vertices[C], pP = this->Mesh.vertices[vIdx];

        // Height checks validating flat geometric conditions
        Eigen::Vector3d baseNormal = (pB - pA).cross(pC - pA);
        double areaSq = baseNormal.squaredNorm();
        if (areaSq < 1e-15) continue;

        baseNormal.normalize();
        double height = std::abs((pP - pA).dot(baseNormal));
        if (height > epsilonDistance) continue; // Skip collapse if umbrella profile is structurally prominent

        // Barycentric inside/outside containment tests verification
        auto barycentric = [](Eigen::Vector3d p, Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c) {
            Eigen::Vector3d v0 = b - a, v1 = c - a, v2 = p - a;
            double d00 = v0.dot(v0), d01 = v0.dot(v1), d11 = v1.dot(v1);
            double d20 = v2.dot(v0), d21 = v2.dot(v1);
            double denom = d00 * d11 - d01 * d01;
            if (std::abs(denom) < 1e-15) return Eigen::Vector3d(-1, -1, -1);
            double v = (d11 * d20 - d01 * d21) / denom;
            double w = (d00 * d21 - d01 * d20) / denom;
            double u = 1.0 - v - w;
            return Eigen::Vector3d(u, v, w);
            };

        Eigen::Vector3d coords = barycentric(pP, pA, pB, pC);
        double eps = 1e-4;
        bool isInside = (coords.x() > -eps && coords.y() > -eps && coords.z() > -eps);

        if (isInside)
        {
            // Set flag to erase the 3 tiny faces sharing the parasitic apex vertex
            for (size_t fIdx : adjFaces) faceRemoved[fIdx] = true;

            // Instantiates a single larger macro triangular base face replacing the collapsed structures
            Eigen::Vector3i macroFace((int)A, (int)B, (int)C);
            cleanFaces.push_back(macroFace);

            vertexModified[vIdx] = true;
            vertexModified[A] = true;
            vertexModified[B] = true;
            vertexModified[C] = true;

            collapseCount++;
        }
    }

    // Reassemble active validated faces sets
    for (size_t i = 0; i < this->Mesh.faces.size(); ++i) {
        if (!faceRemoved[i]) cleanFaces.push_back(this->Mesh.faces[i]);
    }

    this->Mesh.faces = std::move(cleanFaces);
    mesh.faces = this->Mesh.faces;
    mesh.Build_P2P_Neigh();
    mesh.Build_P2F_Neigh();
    mesh.RemoveUnusedVertices(); // Purge orphaned unreferenced vertices indices from tables

    this->Mesh.vertices = std::move(mesh.vertices);
    this->Mesh.faces = std::move(mesh.faces);

    if (collapseCount > 0) {
        std::cout << "\t[KRF - Optimization] Collapsed " << collapseCount << " parasitic valence-3 structures." << std::endl;
    }
}

void AppController::RunTriangleImprovement(int iterations) {
    if (this->Mesh.vertices.empty() || this->Mesh.faces.empty()) return;
    if (!m_scene.m_CadLogic || !m_scene.m_MCGenerator) return;

    NeighborMesh mesh;
    mesh.vertices = this->Mesh.vertices;
    mesh.faces = this->Mesh.faces;

    mesh.buildMeshTopology();
    mesh.ComputeFaceNormals();

    const ImplicitObject& obj = *(m_scene.m_CadLogic);
    double baseDt = 0.1; // Damping constraint step controlling relaxation tracking line search

    // Safe Tangential Laplacian Relaxation pipeline minimizing local geometric noise while preventing model shrinkage
    for (int iter = 0; iter < iterations; ++iter) {
        std::vector<Eigen::Vector3d> nextPositions = mesh.vertices;

        for (int i = 0; i < (int)mesh.vertices.size(); ++i) {
            const std::set<size_t>& neighbors = mesh.P2P_Neigh[i];
            const std::set<size_t>& adjacentFaces = mesh.P2F_Neigh[i];

            if (neighbors.empty()) continue;

            // Extract the uniform local structural neighborhood centroid coordinate
            Eigen::Vector3d centroid(0, 0, 0);
            for (size_t idx : neighbors) {
                centroid += mesh.vertices[idx];
            }
            centroid /= static_cast<double>(neighbors.size());

            Eigen::Vector3d currentP = mesh.vertices[i];
            Eigen::Vector3d n = obj.Gradient(currentP);
            if (n.isZero(1e-12)) continue;
            n.normalize();

            // Project the standard Laplacian shift vector onto the local field tangent plane boundaries
            Eigen::Vector3d shift = centroid - currentP;
            Eigen::Vector3d tangentialShift = shift - shift.dot(n) * n;

            double localDt = baseDt;
            Eigen::Vector3d finalCandidateP = currentP;
            bool moveAccepted = false;

            // Local Backtracking Line Search loop validating topological sanity before allowing moves
            for (int substep = 0; substep < 3; ++substep) {
                Eigen::Vector3d candidateP = currentP + localDt * tangentialShift;
                // Force immediate re-snapping onto the zero-potential isosurface shell
                candidateP = m_scene.m_MCGenerator->ProjectToSurface(obj, candidateP);

                // Normal flipping tracking validation checks
                bool flipDetected = false;
                for (size_t fIdx : adjacentFaces) {
                    Eigen::Vector3i F = mesh.faces[fIdx];
                    Eigen::Vector3d v0 = mesh.vertices[F[0]];
                    Eigen::Vector3d v1 = mesh.vertices[F[1]];
                    Eigen::Vector3d v2 = mesh.vertices[F[2]];
                    Eigen::Vector3d oldNormal = (v1 - v0).cross(v2 - v0);

                    // Substitute vertex with proposition candidate coordinates
                    if (F[0] == i) v0 = candidateP;
                    else if (F[1] == i) v1 = candidateP;
                    else if (F[2] == i) v2 = candidateP;

                    Eigen::Vector3d newNormal = (v1 - v0).cross(v2 - v0);

                    // Dot product check: a negative result implies a destructive face normal flip artifact
                    if (oldNormal.dot(newNormal) <= 0.0) {
                        flipDetected = true;
                        break;
                    }
                }

                if (!flipDetected) {
                    finalCandidateP = candidateP;
                    moveAccepted = true;
                    break; // Proposition validated, skip remaining backtracking passes
                }
                localDt *= 0.5; // Upon flip detection, decrease step length bounds and re-evaluate
            }

            if (moveAccepted) {
                nextPositions[i] = finalCandidateP;
            }
        }
        mesh.vertices = std::move(nextPositions);
    }

    this->Mesh.vertices = std::move(mesh.vertices);
    std::cout << "Tangential Laplacian relaxation completed over " << iterations << " iterations." << std::endl;
}

void AppController::RunSmallTriangleCollapse(double areaThreshold) {
    if (this->Mesh.vertices.empty()) return;

    // Initialize Union-Find / Disjoint-Set flat partitions tracker trees structures
    std::vector<int> parent(this->Mesh.vertices.size());
    for (int i = 0; i < (int)this->Mesh.vertices.size(); ++i) {
        parent[i] = i;
    }

    // Path compression Find implementation guarding against runtime tree degeneration
    auto find = [&](int i, auto& ref_find) -> int {
        if (parent[i] == i) return i;
        return parent[parent[i]] = ref_find(parent[i], ref_find);
        };

    auto unite = [&](int i, int j) {
        int rootI = find(i, find);
        int rootJ = find(j, find);
        if (rootI != rootJ) {
            parent[rootI] = rootJ;
        }
        };

    // Scan faces sets to isolate micro-triangles and schedule edge collapses
    for (const auto& f : this->Mesh.faces) {
        Eigen::Vector3d v0 = this->Mesh.vertices[f[0]];
        Eigen::Vector3d v1 = this->Mesh.vertices[f[1]];
        Eigen::Vector3d v2 = this->Mesh.vertices[f[2]];

        double area = 0.5 * (v1 - v0).cross(v2 - v0).norm();

        if (area < areaThreshold) {
            double d01 = (v1 - v0).squaredNorm();
            double d12 = (v2 - v1).squaredNorm();
            double d20 = (v0 - v2).squaredNorm();

            // Collapse the shortest edge of the degenerate triangle to retain mesh shape integrity
            if (d01 <= d12 && d01 <= d20) unite(f[0], f[1]);
            else if (d12 <= d01 && d12 <= d20) unite(f[1], f[2]);
            else unite(f[2], f[0]);
        }
    }

    // Flatten indices arrays and build clean output vertices pools
    std::vector<Eigen::Vector3d> cleanedVertices;
    cleanedVertices.reserve(this->Mesh.vertices.size());
    std::vector<int> newIndices(this->Mesh.vertices.size(), -1);

    for (int i = 0; i < (int)this->Mesh.vertices.size(); ++i) {
        int root = find(i, find);
        if (newIndices[root] == -1) {
            newIndices[root] = (int)cleanedVertices.size();
            cleanedVertices.push_back(this->Mesh.vertices[root]);
        }
        newIndices[i] = newIndices[root];
    }

    // Filter out faces crushed into singular edges or points by the collapse stage
    std::vector<Eigen::Vector3i> cleanedFaces;
    cleanedFaces.reserve(this->Mesh.faces.size());

    for (const auto& f : this->Mesh.faces) {
        int i0 = newIndices[f[0]];
        int i1 = newIndices[f[1]];
        int i2 = newIndices[f[2]];

        if (i0 != i1 && i1 != i2 && i2 != i0) {
            cleanedFaces.push_back({ i0, i1, i2 });
        }
    }

    size_t initialVertexCount = this->Mesh.vertices.size();
    this->Mesh.vertices = std::move(cleanedVertices);
    this->Mesh.faces = std::move(cleanedFaces);

    size_t removedCount = initialVertexCount - this->Mesh.vertices.size();
    if (removedCount > 0) {
        std::cout << "\t[KRF - Cleanup] Collapsed " << removedCount << " micro-vertices via small triangle removal." << std::endl;
    }
}

void AppController::RunUnifiedCleanup(double epsilon) {
    if (this->Mesh.vertices.empty()) return;

    std::vector<Eigen::Vector3d> cleanedVertices;
    cleanedVertices.reserve(this->Mesh.vertices.size());
    std::vector<int> vertexMap(this->Mesh.vertices.size(), -1);

    // Grid coordinate quantization mapping strings keys to structure cells
    auto hashFunc = [&](const Eigen::Vector3d& p) {
        long x = std::floor(p.x() / epsilon);
        long y = std::floor(p.y() / epsilon);
        long z = std::floor(p.z() / epsilon);
        return std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z);
        };

    std::unordered_map<std::string, int> spatialHash;

    // Weld co-located spatial indices utilizing high-efficiency O(1) hash maps lookups
    for (int i = 0; i < (int)this->Mesh.vertices.size(); ++i) {
        std::string h = hashFunc(this->Mesh.vertices[i]);
        if (spatialHash.count(h)) {
            vertexMap[i] = spatialHash[h];
        }
        else {
            vertexMap[i] = (int)cleanedVertices.size();
            cleanedVertices.push_back(this->Mesh.vertices[i]);
            spatialHash[h] = vertexMap[i];
        }
    }

    std::vector<Eigen::Vector3i> cleanedFaces;
    cleanedFaces.reserve(this->Mesh.faces.size());

    for (const auto& f : this->Mesh.faces) {
        Eigen::Vector3i nf(vertexMap[f[0]], vertexMap[f[1]], vertexMap[f[2]]);

        if (nf[0] == nf[1] || nf[1] == nf[2] || nf[2] == nf[0]) continue;

        // Winding order check guarding against inverted orientation artifacts caused by welding shifts
        Eigen::Vector3d oldN = (this->Mesh.vertices[f[1]] - this->Mesh.vertices[f[0]]).cross(this->Mesh.vertices[f[2]] - this->Mesh.vertices[f[0]]);
        Eigen::Vector3d newN = (cleanedVertices[nf[1]] - cleanedVertices[nf[0]]).cross(cleanedVertices[nf[2]] - cleanedVertices[nf[0]]);

        if (oldN.dot(newN) < 0.0) {
            std::swap(nf[1], nf[2]); // Re-align face normal directions
        }
        cleanedFaces.push_back(nf);
    }

    size_t initialVertexCount = this->Mesh.vertices.size();

    // CRITICAL: Wipe out historical topological tracking states to avoid pointer cross-contamination
    this->Mesh.ClearAll();

    this->Mesh.vertices = std::move(cleanedVertices);
    this->Mesh.faces = std::move(cleanedFaces);

    size_t removedCount = initialVertexCount - this->Mesh.vertices.size();
    if (removedCount > 0) {
        std::cout << "\t[KRF - Cleanup] Welded " << removedCount << " duplicate vertices via Unified Spatial Hashing." << std::endl;
    }
}

void AppController::RunFeaturePreservingSmooth(double sharpAngleDeg, int iterations) {
    if (this->Mesh.vertices.empty() || this->Mesh.faces.empty()) return;
    if (!m_scene.m_CadLogic || !m_scene.m_MCGenerator) return;

    NeighborMesh mesh;
    mesh.vertices = this->Mesh.vertices;
    mesh.faces = this->Mesh.faces;

    mesh.Build_P2P_Neigh();
    mesh.Build_P2F_Neigh();
    mesh.ComputeFaceNormals();

    const double cosThreshold = std::cos(sharpAngleDeg * M_PI / 180.0);
    const ImplicitObject& obj = *(m_scene.m_CadLogic);

    // Anisotropic relaxation pipeline preserving high-curvature ridge boundaries
    for (int iter = 0; iter < iterations; ++iter) {
        std::vector<Eigen::Vector3d> nextPositions = mesh.vertices;

        for (int i = 0; i < (int)mesh.vertices.size(); ++i) {
            const std::set<size_t>& neighbors = mesh.P2P_Neigh[i];
            const std::set<size_t>& adjFaces = mesh.P2F_Neigh[i];

            if (neighbors.empty() || adjFaces.size() < 2) continue;

            // --- SHARP FEATURE IDENTIFICATION CRITERIA ---
            bool isSharp = false;
            // Cross-check face normals across all incident elements sharing node index i
            for (auto it1 = adjFaces.begin(); it1 != adjFaces.end(); ++it1) {
                for (auto it2 = std::next(it1); it2 != adjFaces.end(); ++it2) {
                    if (mesh.face_normals[*it1].dot(mesh.face_normals[*it2]) < cosThreshold) {
                        isSharp = true; // Angle breach detected, structural feature boundary line identified
                        break;
                    }
                }
                if (isSharp) break;
            }

            // Feature lock: freeze the position vector completely if normal angles exceed parameters bounds
            if (isSharp) continue;

            // --- TANGENTIAL APERIODIC SMOOTHING STEP ---
            Eigen::Vector3d currentP = mesh.vertices[i];
            Eigen::Vector3d centroid(0, 0, 0);
            for (size_t idx : neighbors) {
                centroid += mesh.vertices[idx];
            }
            centroid /= static_cast<double>(neighbors.size());

            Eigen::Vector3d n = obj.Gradient(currentP).normalized();
            Eigen::Vector3d shift = centroid - currentP;
            Eigen::Vector3d tangentialShift = shift - shift.dot(n) * n;

            // Drive optimized coordinates adjustments backed by projection tracking loops
            nextPositions[i] = currentP + 0.5 * tangentialShift;
            nextPositions[i] = m_scene.m_MCGenerator->ProjectToSurface(obj, nextPositions[i]);
        }
        mesh.vertices = std::move(nextPositions);

        if (iterations > 1 && iter < iterations - 1) {
            mesh.ComputeFaceNormals(); // Refresh normals references to handle geometric updates
        }
    }

    this->Mesh.ClearAll();
    this->Mesh.vertices = std::move(mesh.vertices);
    this->Mesh.faces = std::move(mesh.faces);

    std::cout << "\t[KRF - Optimization] Feature-preserving smoothing completed ("
        << iterations << " iterations, threshold: " << sharpAngleDeg << " deg)." << std::endl;
}

void AppController::RunUltimateMeshSanitizer(double minAreaThreshold, double minAspectRatio) {
    if (this->Mesh.vertices.empty() || this->Mesh.faces.empty()) return;

    std::vector<bool> faceRemoved(this->Mesh.faces.size(), false);
    std::vector<size_t> remapping(this->Mesh.vertices.size());
    for (size_t i = 0; i < remapping.size(); ++i) {
        remapping[i] = i;
    }

    int sliverCollapses = 0;
    int flatTriangleRemovals = 0;

    // PASS 1: Identify and collapse thin needle-like triangles (Slivers)
    for (size_t fIdx = 0; fIdx < this->Mesh.faces.size(); ++fIdx) {
        const Eigen::Vector3i& f = this->Mesh.faces[fIdx];
        const Eigen::Vector3d& p0 = this->Mesh.vertices[f.x()];
        const Eigen::Vector3d& p1 = this->Mesh.vertices[f.y()];
        const Eigen::Vector3d& p2 = this->Mesh.vertices[f.z()];

        double a = (p1 - p2).norm();
        double b = (p2 - p0).norm();
        double c = (p0 - p1).norm();
        double perimeter = a + b + c;

        if (perimeter < 1e-12) {
            faceRemoved[fIdx] = true;
            continue;
        }

        Eigen::Vector3d crossProd = (p1 - p0).cross(p2 - p0);
        double area = 0.5 * crossProd.norm();

        // Shape metric: Inradius r = 2 * Area / Perimeter
        double inradius = (2.0 * area) / perimeter;
        double maxEdge = std::max({ a, b, c });

        if (maxEdge < 1e-12) continue;
        double aspectRatio = inradius / maxEdge; // Normalized triangle shape aspect ratio

        // Threshold checks: schedule collapse if area is degenerate OR shape profiles are dangerously flat
        if (area < minAreaThreshold || aspectRatio < minAspectRatio) {
            faceRemoved[fIdx] = true;
            size_t vToCollapse = f.x();
            size_t vTarget = f.y();

            // Isolate the shortest edge vector components to direct the collapse orientation path
            if (a < b && a < c) {
                vToCollapse = f.z();
                vTarget = f.y();
            }
            else if (b < a && b < c) {
                vToCollapse = f.z();
                vTarget = f.x();
            }
            else {
                vToCollapse = f.y();
                vTarget = f.x();
            }

            remapping[vToCollapse] = vTarget;
            sliverCollapses++;
        }
    }

    // Resolve structural remapping dependency loops sequentially
    for (size_t i = 0; i < remapping.size(); ++i) {
        size_t target = remapping[i];
        int safety = 0;
        while (remapping[target] != target && safety < 5) {
            target = remapping[target];
            safety++;
        }
        remapping[i] = target;
    }

    // PASS 2: Reconstruct output face lists filtering out flat and colinear geometries configurations
    std::vector<Eigen::Vector3i> cleanFaces;
    cleanFaces.reserve(this->Mesh.faces.size());

    for (size_t fIdx = 0; fIdx < this->Mesh.faces.size(); ++fIdx) {
        if (faceRemoved[fIdx]) continue;

        const Eigen::Vector3i& f = this->Mesh.faces[fIdx];
        size_t v0 = remapping[(size_t)f.x()];
        size_t v1 = remapping[(size_t)f.y()];
        size_t v2 = remapping[(size_t)f.z()];

        if (v0 == v1 || v1 == v2 || v2 == v0) {
            flatTriangleRemovals++;
            continue;
        }

        // Geometric boundary checks evaluating internal angles limits
        Eigen::Vector3d e1 = (this->Mesh.vertices[v1] - this->Mesh.vertices[v0]).normalized();
        Eigen::Vector3d e2 = (this->Mesh.vertices[v2] - this->Mesh.vertices[v0]).normalized();

        // Discard face elements if directional alignment implies near-total colinearity
        if (std::abs(e1.dot(e2)) > 0.999) {
            flatTriangleRemovals++;
            continue;
        }
        cleanFaces.push_back({ (int)v0, (int)v1, (int)v2 });
    }

    // Pack geometry pools and drop detached non-referenced vertex structures
    NeighborMesh mesh;
    mesh.vertices = std::move(this->Mesh.vertices);
    mesh.faces = std::move(cleanFaces);
    mesh.Build_P2P_Neigh();
    mesh.Build_P2F_Neigh();
    std::cout << "\t";
    mesh.RemoveUnusedVertices();

    this->Mesh.ClearAll();
    this->Mesh.vertices = std::move(mesh.vertices);
    this->Mesh.faces = std::move(mesh.faces);

    if (sliverCollapses > 0 || flatTriangleRemovals > 0) {
        std::cout << "\t[KRF Sanitizer] Removed " << sliverCollapses
            << " slivers via edge-collapse and purged " << flatTriangleRemovals
            << " flat face configurations." << std::endl;
    }
}

void AppController::attachPrimitives() {
    // Collect analytical base primitive addresses to feed the polymorphic R-function execution engine core
    m_scene.m_CadLogic->primitives.clear();
    for (const auto& obj : m_objects) {
        m_scene.m_CadLogic->primitives.push_back(obj.get());
    }
}

Eigen::Vector3d AppController::doubleToColorDiscreteUpdated(const double& d, const double& ncol, const GLScene& scene) {
    double t = 0;
    const double g_max = scene.GetMax();
    const double g_min = scene.GetMin();
    const auto& cp = scene.m_palette;

    if (d > 0) {
        t = (g_max > 1e-7) ? std::clamp(d / g_max, 0.0, 1.0) : 0.0;
        t = std::floor(t * ncol) / ncol;
        return (1.0 - t) * cp.posStart + t * cp.posEnd;
    }
    else {
        t = (std::abs(g_min) > 1e-7) ? std::clamp(d / g_min, 0.0, 1.0) : 0.0;
        t = std::floor(t * ncol) / ncol;
        return (1.0 - t) * cp.negStart + t * cp.negEnd;
    }
}

Eigen::Vector3d AppController::doubleToColorDiscreteUpdatedLocal(const double& d, const double& ncol, double fmin, double fmax) {
    double t = 0;
    const double g_max = fmax;
    const double g_min = fmin;
    const auto& cp = m_scene.m_palette;

    if (d > 0) {
        t = (g_max > 1e-7) ? std::clamp(d / g_max, 0.0, 1.0) : 0.0;
        t = std::floor(t * ncol) / ncol;
        return (1.0 - t) * cp.posStart + t * cp.posEnd;
    }
    else {
        t = (std::abs(g_min) > 1e-7) ? std::clamp(d / g_min, 0.0, 1.0) : 0.0;
        t = std::floor(t * ncol) / ncol;
        return (1.0 - t) * cp.negStart + t * cp.negEnd;
    }
}


/**
 * @brief Executes a performance and computational efficiency benchmark comparing
 * higher-order N-ary composition methods.
 * @details Generates hyper-dimensional random evaluation sets to stress-test
 * the recursive ternary spherical tree structure against direct multi-dimensional formulations.
 * * @param dim_start The target dimensionality (N) of the input potential field vectors.
 * @param point_number Total number of hyper-dimensional sample configurations evaluated per pass.
 */
void AppController::ComparisonBenchmark(int dim_start, int point_number) {
    std::cout << "\n==========================================================" << std::endl;
    std::cout << "  R-FUNCTION PERFORMANCE BENCHMARK (N=" << dim_start << ")" << std::endl;
    std::cout << "==========================================================" << std::endl;

    // Phase 1: Pre-allocate target evaluation arrays to prevent memory allocation noise during timing loops
    std::vector<Eigen::VectorXd> testPoints;
    testPoints.reserve(point_number);
    for (int i = 0; i < point_number; ++i) {
        testPoints.push_back(Eigen::VectorXd::Random(dim_start));
    }

    // Phase 2: Initialize logical operators and state tracking attributes
    RCore::SphericalEvaluator spherical;
    spherical.Init(3); // The directional spherical operator tracks evaluation steps inside 3D blocks
    bool isInter = true;
    double dummy_sum = 0.0; // Statistical checksum accumulator ensuring total evaluation loops validity

    // Phase 3: Instantiate the performance metric measurement wrapper utilizing lambda function tracking loops
    auto measure = [&](const std::string& name, auto func) {
        auto start = std::chrono::high_resolution_clock::now();

        // Main tight execution loop driving sequential field evaluations over the sampling vector pool
        for (const auto& X : testPoints) {
            dummy_sum += func(X, isInter);
        }

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end - start;

        // Print standardized performance logging tracks output formatting counters
        std::cout << std::left << std::setw(30) << name << ": "
            << std::fixed << std::setprecision(4) << duration.count() << " ms "
            << "(" << (duration.count() * 1e6 / point_number) << " ns/eval)" << std::endl;
        };

    // Phase 4: Execute sequential benchmarking steps tracking logic composition methods efficiency
    measure("Spherical (Proposed Ternary)", [&](const Eigen::VectorXd& X, bool inter) {
        return EvaluateRecursiveSpherical(X, inter, spherical);
        });

    // measure("Zenkin Nd (Alternating Sum)", [](const Eigen::VectorXd& X, bool inter) {
    //     return RCore::ZenkinNd(X, inter);
    // });

    measure("Rvachev Nd (Linear m=1)", [](const Eigen::VectorXd& X, bool inter) {
        return RCore::RvachevNd(X, inter, 1);
        });

    // Phase 5: Flush structural checks logs insuring execution integrity against compiler pruning
    std::cout << "----------------------------------------------------------" << std::endl;
    std::cout << "Checksum: " << dummy_sum << std::endl;
    std::cout << "==========================================================\n" << std::endl;
}


void AppController::makePolygon(int n_sides, const double radius) {
    // Regular Polygon Generation (e.g., n_sides = 5 for a pentagon)
    m_objects.clear();

    const double PI = std::acos(-1.0);

    std::vector<Eigen::Vector3d> vertices;
    for (int i = 0; i < n_sides; ++i) {
        double theta = 2.0 * PI * i / n_sides;
        vertices.push_back(Eigen::Vector3d(radius * std::cos(theta), radius * std::sin(theta), 0.0));
    }

    for (int i = 0; i < n_sides; ++i) {
        // Connect vertex i to vertex i+1 (with wrap around for the last side)
        const auto& pStart = vertices[i];
        const auto& pEnd = vertices[(i + 1) % n_sides];

        m_objects.push_back(std::make_shared<ImplicitPlane>(pStart, pEnd));
    }
}

