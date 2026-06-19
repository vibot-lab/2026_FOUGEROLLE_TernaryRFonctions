/**
 * @file GLScene.h
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

#pragma once

#include <GL/glut.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>

 // Specialized framework inclusions for CAD and KRF geometric tracking
#include "CAD_Scene.h"
#include "MarchingCubes.h"
#include "NeighborMesh.h"

// Numerical compatibility layer ensuring standard macro definitions for math constants
#ifndef M_PI
#define _USE_MATH_DEFINES
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#endif

/**
 * @brief Handles the interactive OpenGL contextual visualization workspace.
 * @details Manages trackball rotations, discrete dual-palette colormaps, viewport bounding ranges,
 * and holds runtime lifecycle references for implicit primitives and meshing generators.
 */
class GLScene {
public:
    // =========================================================================
    // NESTED CONFIGURATION STRUCTURES
    // =========================================================================

    /**
     * @brief Spatial domain geometry configuration bounding the evaluation plane.
     */
    struct DomainConfig {
        Eigen::Vector3d U = Eigen::Vector3d::UnitX();     ///< Primary planar horizontal direction vector
        Eigen::Vector3d V = Eigen::Vector3d::UnitY();     ///< Secondary planar vertical direction vector
        Eigen::Vector3d LC = Eigen::Vector3d(-2.0, -2.0, 0.0); ///< Local coordinate system center origin translation offset
        double scale = 4.0;                               ///< Uniform spatial scaling dimension factor
    };

    /**
     * @brief Continuous scalar field discrete mapping color boundaries.
     */
    struct ColorPalette {
        Eigen::Vector3d posStart = { 0.75, 0.0, 0.0 };    ///< Deep red anchor for lower positive potential bands
        Eigen::Vector3d posEnd = { 1.0, 0.75, 0.25 };   ///< Yellow-orange boundary for peak positive field zones
        Eigen::Vector3d negStart = { 0.0, 0.0, 1.0 };     ///< Pure blue anchor for lower negative potential levels
        Eigen::Vector3d negEnd = { 0.0, 0.0, 0.0 };     ///< Black core transition for severe negative potentials
    };

    // =========================================================================
    // CORE LIFECYCLE & RENDER SETUP
    // =========================================================================
    GLScene();
    void setLights();
    void Init(int& argc, char** argv);

    // =========================================================================
    // KRF CORE RESEARCH & GEOMETRIC POINTERS PIPELINE
    // =========================================================================

    std::unique_ptr<CAD_Scene> m_CadLogic;              ///< Abstract composition manager handling higher-order R-functions configurations
    std::unique_ptr<MarchingCubes> m_MCGenerator;       ///< Isosurface simplicial sampler compiling raw implicit bounds
    NeighborMesh m_Mesh;                                ///< Manifold mesh tracking entity storing simplicial topological structures
    std::vector<std::shared_ptr<ImplicitObject>> m_Primitives; ///< Runtime memory containment pool managing analytical tracking shapes

    // =========================================================================
    // SCALAR BOUNDING RANGES & PALETTE SYNC CONTROLS
    // =========================================================================

    double global_min;                                  ///< Lowest potential value encountered across global grid evaluation passes
    double global_max;                                  ///< Highest potential value encountered across global grid evaluation passes
    std::vector<Eigen::Vector2d> local_min_max;         ///< Per-method operational field scale boundaries arrays

    void UpdateRange(double val);
    void DrawAxes();

    // =========================================================================
    // STRUCTURAL GRAPHICAL PRIMITIVES RENDERING METHODS
    // =========================================================================

    /** @brief Renders the pre-compiled OpenGL active display list. */
    void RenderGlobalMesh(int listId);

    /**
     * @brief Overlay routine rendering structural dashed guidance rings at the z=0 cut plane.
     */
    void DrawSphereContour(
        const Eigen::Vector3d& center,
        double radius,
        const Eigen::Vector3d& color = Eigen::Vector3d(1, 1, 1)
    );

    /**
     * @brief Standard geometric glyph vector visualization for mathematical field verification.
     */
    void RenderGradientGlyphs(
        const std::vector<Eigen::Vector3d>& points,
        const std::vector<Eigen::VectorXd>& gradients,
        const Eigen::Vector3d& color,
        double scale = 0.1
    );

    // =========================================================================
    // INTERACTIVE CONTROLS, TRACKBALL & STATE FLAGS
    // =========================================================================

    int lights_on;                                      ///< System state flag tracking illumination activation states
    int window_width;                                   ///< Current horizontal resolution aspect of the main window viewport
    int window_height;                                  ///< Current vertical resolution aspect of the main window viewport
    Eigen::Vector3d Object_Move;                        ///< Spatial tracking translation vectors (X, Y, Zoom-Z)

    // Virtual trackball transformation arrays
    Eigen::Matrix4d rotation_matrix;                    ///< Cumulative orientation matrix tracker capturing drag updates
    Eigen::Vector2i previous_mouse_position;            ///< Historical tracking pixel step location
    bool is_rotating;                                   ///< State condition tracker determining active cursor drag rotations

    // Active spatial layout and mapping palettes
    DomainConfig m_domain;                              ///< Viewport bounds domain settings references
    ColorPalette m_palette;                             ///< Color mapping parameters lookup arrays


    Eigen::Vector3d camPos, camTarget;                  ///< Camera position and target 
    // Matrix cache storing compiled dual logic indices (x: Intersection list, y: Union list) per row
    std::vector<Eigen::Vector2i> RF_ogl_lists;

    // =========================================================================
    // INLINE VALUE CONTROLLERS & EXPLICIT GETTERS
    // =========================================================================

    /**
     * @brief Resets boundaries counters to inverted extreme ranges to welcome clean tracking passes.
     */
    void ResetRange() {
        global_min = 1e10;
        global_max = -1e10;
    }

    double GetMin() const { return global_min; }        ///< Returns the processed global minimum field potential metric
    double GetMax() const { return global_max; }        ///< Returns the processed global maximum field potential metric
};