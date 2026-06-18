
/**
 * @file AppController.h
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

//Standard headers
#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <string>
#include <utility>
#include <set>
#include <map>

//Eigen headers
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

//Local headers
#include "GLScene.h"
#include "Rfunctions.h"
#include "ImplicitObjects.h"
#include "NeighborMesh.h"

 /**
  * @brief Defines the structural type for branching implicit multi-junction scenes for mode == CAD_JUNCTION
  */
enum class JunctionType {
    X_JUNCTION,
    Y_JUNCTION
};

/**
 * @brief Core application controller implementing the state machine, numerical benchmarks,
 * visualization setups, and the mesh optimization pipeline.
 */
class AppController {
public:
    /**
     * @brief Operational states governing GUI interactions, rendering, and processing loops.
     */
    enum class SceneMode {
        ClassicalExample,                ///< Classical mode: renders only ternary R-functions over a planar domain with 3 spheres
        BenchmarkGradient,               ///< Benchmarks gradient field accuracy, efficiency, and robustness
        BenchmarkGradientUnbalancedTree, ///< Benchmarks efficiency comparison between different evaluation trees
        NaryExample,                     ///< Meshes, renders, and compares higher-order N-ary R-functions
        CADjunction                      ///< Executes Marching Cubes and mesh rendering for triple junction results
    };

    

    // Data for Rfunction evaluation and rendering
    // heavy structure, but guarantees the sync between all points and their various implicit values
    // each point (3d) --> an array of pairs (one pair for each implicit function)
    // first element corresponds to Conjunction / Intersection
    // second element corresponds to Disjunction / Union

    std::vector< std::pair<Eigen::Vector3d, std::vector <Eigen::Vector2d> > > data;

    // =========================================================================
    // VALUE MEMBERS, STATE FLAGS & BUFFERS (DECLARATIONS FIRST)
    // =========================================================================

    // Application States & Behavioral Parameters
    SceneMode currentMode = SceneMode::ClassicalExample; ///< Current active application behavioral mode
    int RFnumber = 8;         ///< Number of R-functions to be evaluated (allocates RFnumber x 2 OpenGL lists)
    int m_functionUsed;       ///< Target R-function currently used: from 0 to RFnumber - 1
    int Rvachev_m = 1;        ///< The 'm' regularization parameter for Rvachev N-ary R-functions
    double epsilon_crit = 1.0; ///< Critical threshold parameter used for convexity boundary tests

    // domain definition for the epsilon values. 
    double convex_min, convex_max;
    int sample_convex;

    // Core Mesh & Functional Evaluators
    NeighborMesh Mesh;                      ///< Attribute member to store geometry, connectivity and topology
    RCore::SphericalEvaluator SpherEval;    ///< Spherical core functional evaluator
    GLScene m_scene;                        ///< Global opengGL scene contextual reference
    std::vector<std::shared_ptr<ImplicitObject>> m_objects; ///< Vector pool of instantiated primitives
    RCore::SphericalEvaluator m_evaluator;                 ///< Modular testing spherical evaluator

    // Graphical State Flags & OpenGL References
    bool gradients = false;          ///< Active flag for tracking gradient field rendering
    int active_list_idx;             ///< Render target list index linked to active R-function
    int m_activeMethod = 0;
    int m_activeMeshId;              ///< Target OpenGL list reference ID
    bool m_isIntersection;           ///< Toggle condition flag between Union and Intersection
    bool m_is3D;                     ///< Defines flat or perspective spatial rendering
    int m_sampling;                  ///< Grid sampling density for rendering passes

    // Core Evaluation Data Cache Buffers
    std::vector<Eigen::Vector3d> EvaluatedPoints;    ///< Coordinate points sampled over the grid

    // Potential evaluation caches ==> not used but ready to store values if needed
    std::vector<double> RpBinaryTree;
    std::vector<double> R0mBinaryTree;
    std::vector<double> F_RvachevNd;
    std::vector<double> F_ZenkinNd;
    std::vector<double> F_Sph_Directional;
    std::vector<double> F_Sph_Normalized;

    // Associated structural gradient vectors
    std::vector<Eigen::VectorXd> G_RpBinaryTree;
    std::vector<Eigen::VectorXd> G_R0mBinaryTree;
    std::vector<Eigen::VectorXd> G_RvachevNd;
    std::vector<Eigen::VectorXd> G_ZenkinNd;
    std::vector<Eigen::VectorXd> G_Sph_Directional;
    std::vector<Eigen::VectorXd> G_Sph_Normalized;

    // =========================================================================
    // LIFECYCLE & MEMBER FUNCTIONS (PROTOTYPES)
    // =========================================================================
    AppController();
    ~AppController() = default;

    // Singleton pattern utility accessor
    AppController& getInstance() { return *this; }

    // Marching Cubes & Mesh Optimization Core Pipeline
    void RunMarchingCube(const Eigen::Vector3d& minBound, const Eigen::Vector3d& maxBound, int gridResolution);
    void RunTopologyCleanup(double epsilon);
    int RunValenceOptimization(int maxPasses = 5);
    void RunParasiteCollapse(double epsilonDistance);
    void RunTriangleImprovement(int iterations = 50);
    void RunSmallTriangleCollapse(double areaThreshold);
    void RunUnifiedCleanup(double epsilon);
    void RunFeaturePreservingSmooth(double sharpAngleDeg, int iterations = 10);
    void RunUltimateMeshSanitizer(double minAreaThreshold, double minAspectRatio);

    // Sampling Generators & Grid Query Routines
    std::vector<std::shared_ptr<ImplicitObject>> MakeDaisy(double R = 1.0, double Rc = 0.5, double numPrimitives = 7);
    std::vector<std::shared_ptr<ImplicitObject>> MakeDaisyConvexTest(double& epsilonCrit, int primitiveNumber, double R = 1.0, double Rc = 0.5);

    // Grid evaluation for all possible Rfunctions
    std::vector<std::pair<Eigen::Vector3d, std::vector<Eigen::Vector2d>>> ComputeFullGridData(unsigned int sampling);
    std::vector<std::pair<Eigen::Vector3d, std::vector<Eigen::Vector2d>>> ComputeFullGridDataConvexTests(
        unsigned int sampling,
        double eps_min,
        double eps_max,
        int eps_sample
    );

    // Performance Benchmarks & Scientific Evaluation
    void RunFullBenchmark();
    void RunNewtonRaphsonBenchmark(int gridRes);
    void ComparisonBenchmark(int dim_start, int point_number);
    void BenchmarkGradient(bool isInter, int N, double field_value_start, double field_value_end, int number_of_runs);
    void BenchmarkGradientUnbalancedTree(bool isInter, int N, double field_value_start, double field_value_end, int number_of_runs);
    double EvaluateRecursiveSpherical(const Eigen::VectorXd& X, bool inter, RCore::SphericalEvaluator& eval);

    // Color Palette Related Functions for color coding Rfunction values
    Eigen::Vector3d doubleToColorDiscreteUpdated(const double& d, const double& ncol, const GLScene& scene);
    Eigen::Vector3d doubleToColorDiscreteUpdatedLocal(const double& d, const double& ncol, double fmin, double fmax);

    // OpenGL Framework & Callbacks Handling
    void Init(int& argc, char** argv);
    void Run();
    static void DisplayCallback();
    static void ReshapeCallback(int w, int h);
    static void KeyboardCallback(unsigned char key, int x, int y);
    static void MouseCallback(int button, int state, int x, int y);
    static void MotionCallback(int x, int y);
   
    // =========================================================================
    // RUNTIME RUN-LOG ROUTINES & CONSOLE TRACKERS
    // =========================================================================

    std::string getFunctionName(int n) {
        if (currentMode == AppController::SceneMode::ClassicalExample
            ||
            currentMode == AppController::SceneMode::NaryExample)
            switch (n) {
            case 0: return std::string("Min/Max");
            case 1: return std::string("Chained Rp");
            case 2: return std::string("Chained R0m");
            case 3: return std::string("Zenkin Nd");
            case 4: return std::string("Rvachev Nd");
            case 5: return std::string("Spherical directional");
            case 6: return std::string("Spherical normalized");
            case 7: return std::string("Nd tests");
            default: return std::string("Undefined function");

            }

       
        return std::string("Undefined Mode");
    }

    // Mutators, Sync, & Drawing Controls
    void updateActiveListIndex();
    void ComputeAndBufferMesh(unsigned int sampling);
    void set_isInter(bool b) { m_isIntersection = b; }
    bool get_isInter() const { return m_isIntersection; }
    void setLogic(LogicType logic) { m_scene.m_CadLogic->currentLogic = logic; }
    
    //tranfer function pointers from array of implicit objects to CAD_Logic instance in charge of evaluating the global Rfunction
    void attachPrimitives();

    //implicit scene construction methods:
    void makeSpheres();
    void makePolygon(int n_sides = 5, const double radius = 1.0);

    // =========================================================================
    // INLINE FUNCTIONS IMPLEMENTATIONS 
    // =========================================================================

    /**
     * @brief Parallelized multi-dimensional Newton-Raphson projector to track and snap points onto the zero-set surface.
     */
    void AdjustPoints(const ImplicitObject& obj, std::vector<Eigen::Vector3d>& vertices)
    {
        // Thread-safe parallelization across all vertices using OpenMP
#pragma omp parallel for 
        for (int i = 0; i < (int)vertices.size(); ++i) {
            Eigen::Vector3d& P = vertices[i];

            // Local Newton-Raphson optimization loop (maximum of 5 tracking steps)
            for (int iter = 0; iter < 5; ++iter) {
                // Evaluate the field value (potential/distance to the isosurface f(P) = 0)
                double val = obj.Evaluate(P);

                // Convergence criteria: stop if the point is close enough to the isosurface
                if (std::abs(val) < 1e-7) break;

                // Compute the spatial gradient vector at the current position
                Eigen::Vector3d grad = obj.Gradient(P);
                double g2 = grad.squaredNorm();

                // Numerical safety check: prevent division by zero near critical points or flat regions
                if (g2 < 1e-8) break;

                // Project the point along the normal direction using the Newton-Raphson step formula
                P -= (val / g2) * grad;
            }
        }
    }

    /**
     * @brief Assembles analytical cylinders to define intersection branch scenes (X or Y configurations).
     */
    std::vector<std::shared_ptr<ImplicitObject>> MakeTripleJunctionScene(JunctionType type, double radius = 0.5) {
        std::vector<std::shared_ptr<ImplicitObject>> objects;

        if (type == JunctionType::X_JUNCTION) {
            objects.push_back(std::make_shared<ImplicitCylinder>(Eigen::Vector3d(-1, 0, 0), Eigen::Vector3d(1, 0, 0), radius));
            objects.push_back(std::make_shared<ImplicitCylinder>(Eigen::Vector3d(0, -1, 0), Eigen::Vector3d(0, 1, 0), radius));
            objects.push_back(std::make_shared<ImplicitCylinder>(Eigen::Vector3d(0, 0, -1), Eigen::Vector3d(0, 0, 1), radius));
        }
        else if (type == JunctionType::Y_JUNCTION) {
            double eps_offset = -1e-6;
            for (int i = 0; i < 3; ++i) {
                double theta = (M_PI / 3.0) + (i * 2.0 * M_PI / 3.0);
                Eigen::Vector3d direction(std::cos(theta), std::sin(theta), 0);
                objects.push_back(std::make_shared<ImplicitCylinder>(eps_offset * direction, direction, radius));
            }
        }

        return objects;
    }

    /**
     * @brief Resets and purges all local data vectors to prevent cross-contamination.
     */
    void cleanAlldata() {
        EvaluatedPoints.clear();
        RpBinaryTree.clear();
        R0mBinaryTree.clear();
        F_RvachevNd.clear();
        F_ZenkinNd.clear();
        F_Sph_Directional.clear();
        F_Sph_Normalized.clear();
        G_RpBinaryTree.clear();
        G_R0mBinaryTree.clear();
        G_RvachevNd.clear();
        G_ZenkinNd.clear();
        G_Sph_Directional.clear();
        G_Sph_Normalized.clear();
    }

private:
    // File exporter utility
    void SaveResultsToCSV(const std::vector<BenchResult>& results);

    // Static runtime context singleton tracking pointer
    static AppController* s_instance;
};