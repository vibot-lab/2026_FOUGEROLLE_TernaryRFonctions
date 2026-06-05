/**
 * @file main.cpp
 * @author Yohan FOUGEROLLE
 * @date 2026
 * @brief Main entry for several usages of Rfunctions -- simply uncomment the main file to be run
 */

#include <iostream>
#include <memory>
#include "AppController.h"
#include "Rfunctions.h"
#include "MarchingCubes.h"

// =========================================================================
// main program for triple junction surface generation using marching cube + visualisation
// =========================================================================


int main(int argc, char** argv) {

    // =========================================================================
    // INITIALIZATION & PARAMETERS CONFIGURATION
    // =========================================================================

    // unique instance of controller application that accesses everything
    AppController app;

    //current Mode controls how the program reacts (keybord, rendering, etc.)
    app.currentMode = AppController::SceneMode::CADjunction;  // used for marching cube rendering

    //OpenGL initialisation
    app.Init(argc, argv);

    // Marching Cubes bounding box definition
    Eigen::Vector3d minBound(-1.5, -1.5, -1.5);
    Eigen::Vector3d maxBound(1.5, 1.5, 1.5);
    int gridResolution = 80; // Standard testing range: 50, 70, 80, 100, 120
    double dx = (maxBound.x() - minBound.x()) / static_cast<double>(gridResolution);

    // Adaptive thresholds for sequential mesh optimization stages
    // Initial (robust) coefficients used during rough topology creation
    double initEpsilon = dx * 0.01;                        // 1% of voxel cell length for vertex welding
    double areaThreshold = initEpsilon * initEpsilon * 5;  // Upper bound for degenerate triangle area

    // Final polishing coefficients to fine-tune the output simplicial complex
    // Keeps precision aligned with initial grid constraints to avoid unexpected deformations
    double finalEpsilon = initEpsilon * 2.0;
    double finalAreaThreshold = finalEpsilon * finalEpsilon;

    // Feature detection parameter
    double sharpAngleThreshold = 20.0;

    // =========================================================================
    // SCENE ASSEMBLY & OPERATOR BINDING
    // =========================================================================

    // Assign ternary spherical operator pointers to both mesh and CAD logic structures
    // so that the function can be called from both instances
    app.Mesh.TernarySphericalOperator = &(app.SpherEval);
    app.m_scene.m_CadLogic->TernarySphericalOperator = &(app.SpherEval);

    // Set implicit logical layout (false: global union of primitives)
    app.m_scene.m_CadLogic->setLogicalOperation(false);

    // Populate composition objects at the controller level
    app.m_objects = app.MakeTripleJunctionScene(JunctionType::X_JUNCTION);
    //app.m_objects = app.MakeTripleJunctionScene(JunctionType::Y_JUNCTION);


    // Attach implicit primitives to the CAD_Logic member that is in charge of computing the global Rfunction
    app.attachPrimitives();

    // Select the formulation family for multi-primitive evaluation:
    // Available: TERNARY_KRF (Spherical), BINARY_RP (Binary combinations), MIN_NAIVE (Classical Min/Max)
    app.setLogic(LogicType::TERNARY_KRF);

    // =========================================================================
    // SURFACE CONSTRUCTION AND OPTIMIZATION PIPELINE
    // =========================================================================

    // -------------------------------------------------------------------------
    // PHASE 1: ISOSURFACE GENERATION & WATERTIGHT COMPLEX ASSEMBLY
    // -------------------------------------------------------------------------
    std::cout << "[PHASE 1] Isosurface Extraction & Initial Cleaning" << std::endl;

    std::cout << "\t-> Running Marching Cubes..." << std::endl;
    app.RunMarchingCube(minBound, maxBound, gridResolution);

    std::cout << "\t-> Performing close vertices welding..." << std::endl;
    app.Mesh.WeldVertices(initEpsilon);

    std::cout << "\t-> Running topological detoxification (Immediate cleanup)..." << std::endl;
    app.RunTopologyCleanup(initEpsilon);

    std::cout << "\t-> Refreshing global mesh adjacency data..." << std::endl;
    app.Mesh.buildMeshTopology();

    // -------------------------------------------------------------------------
    // PHASE 2: TOPOLOGICAL OPTIMIZATION (VALENCE REGULARIZATION)
    // -------------------------------------------------------------------------
    std::cout << "\n[PHASE 2] Topological Regularization (Isotropic Optimization)" << std::endl;

    std::cout << "\t-> Maximizing valence-6 configurations via strict Independent Set Edge Flips..." << std::endl;
    int totalFlips = app.RunValenceOptimization();
    std::cout << "\t   [Success] Topology optimized with " << totalFlips << " edge flips." << std::endl;

    std::cout << "\t-> Performing surgical collapse of parasitic valence-3 umbrellas..." << std::endl;
    app.RunParasiteCollapse(initEpsilon);

    std::cout << "\t-> Synchronizing mesh topology post-collapse..." << std::endl;
    app.Mesh.buildMeshTopology();

    // -------------------------------------------------------------------------
    // PHASE 3: GEOMETRIC OPTIMIZATION & TANGENTIAL RELAXATION
    // -------------------------------------------------------------------------
    std::cout << "\n[PHASE 3] Geometric Relaxation & Feature Alignment" << std::endl;

    std::cout << "\t-> Running safe tangential Laplacian smoothing..." << std::endl;
    app.RunTriangleImprovement();

    std::cout << "\t-> Snapping vertices back onto the KRF isosurface via parallel Newton-Raphson tracking..." << std::endl;
    app.AdjustPoints(*(app.m_scene.m_CadLogic), app.Mesh.vertices);

    std::cout << "\t-> Rebuilding reference topology for sharp feature extraction..." << std::endl;
    app.Mesh.buildMeshTopology();

    // -------------------------------------------------------------------------
    // PHASE 4: SHARP FEATURE REFINEMENT
    // -------------------------------------------------------------------------
    std::cout << "\n[PHASE 4] Sharp Edges Geometric Refinement" << std::endl;
    std::cout << "\t-> Aligning sampling density across high-curvature ridges..." << std::endl;
    app.m_scene.m_MCGenerator->RefineSharpEdges(*(app.m_scene.m_CadLogic), app.Mesh, sharpAngleThreshold);

    // -------------------------------------------------------------------------
    // PHASE 5: FINAL MESH POLISHING & SANITIZATION
    // -------------------------------------------------------------------------
    std::cout << "\n[PHASE 5] Final Multi-Criteria Polishing" << std::endl;

    std::cout << "\t-> Gathering initial geometric statistics:" << std::endl;
    app.Mesh.GetGeometricStatistics();

    std::cout << "\n\t-> Eliminating micro-triangles via bounded Disjoint-Set edge collapse..." << std::endl;
    app.RunSmallTriangleCollapse(finalAreaThreshold);

    std::cout << "\t-> Executing final spatial hashing vertex welding..." << std::endl;
    app.RunUnifiedCleanup(finalEpsilon);

    std::cout << "\t-> Running feature-preserving anisotropic smoothing..." << std::endl;
    app.RunFeaturePreservingSmooth(sharpAngleThreshold);

    std::cout << "\t-> Running ultimate mesh sanitizer against high aspect ratio sliver configurations..." << std::endl;
    app.RunUltimateMeshSanitizer(finalAreaThreshold * 2, 0.06);

    // -------------------------------------------------------------------------
    // PHASE 6: FINAL RECONSTRUCTION & GPU BUFFERING
    // -------------------------------------------------------------------------
    std::cout << "\n[PHASE 6] Final Reconstruction & GPU Buffering" << std::endl;

    app.Mesh.buildMeshTopology();

    std::cout << "\t-> Gathering optimized geometric statistics:" << std::endl;
    app.Mesh.GetGeometricStatistics();

    std::cout << "\n\t-> Packing arrays into graphic rendering buffers..." << std::endl;
    app.ComputeAndBufferMesh(100);

    std::cout << "[SUCCESS] Final KRF Mesh compiled successfully: " << app.Mesh.faces.size() << " triangles." << std::endl;

    // Launch application window loop (OpenGL / freeglut context visualization)

    app.Run();

    return 0;
}




/*
// =========================================================================
// main program for benchmarking triple junction surface generation using marching cube
// =========================================================================

int main(int argc, char** argv) {
    AppController app;

    // =========================================================================
    // APPLICATION INITIALIZATION & CONFIGURATION
    // =========================================================================

    // Initialize application context (OpenGL, Scene structures, etc.)
    app.Init(argc, argv);
    app.currentMode = AppController::SceneMode::CADjunction;

    // Assign ternary spherical operator pointers to both mesh and CAD logic structures
    app.Mesh.TernarySphericalOperator = &(app.SpherEval);
    app.m_scene.m_CadLogic->TernarySphericalOperator = &(app.SpherEval);

    // Set implicit logical layout (false: global union of primitives)
    app.m_scene.m_CadLogic->setLogicalOperation(false);

    // Populate composition objects at the controller level
    app.m_objects = app.MakeTripleJunctionScene(JunctionType::X_JUNCTION);

    // Extract raw pointers to feed the polymorphic R-function evaluation core
    app.m_scene.m_CadLogic->primitives.clear();
    for (const auto& obj : app.m_objects) {
        app.m_scene.m_CadLogic->primitives.push_back(obj.get());
    }

    // =========================================================================
    // COMPARATIVE BENCHMARK EXECUTION
    // =========================================================================

    // This function automatically evaluates the 3 logic formulations 
    // (TERNARY_KRF, BINARY_RP, MIN_NAIVE) across resolutions ranging from 50 to 120,
    // applying the full optimization pipeline (Relaxation, Sharp Refine, Polishing).
    std::cout << "====================================================" << std::endl;
    std::cout << "Starting CAD Junction Robustness Benchmark" << std::endl;
    std::cout << "====================================================" << std::endl;

    app.RunFullBenchmark();

    std::cout << "====================================================" << std::endl;
    std::cout << "Benchmark Finished. Check results.csv for analysis." << std::endl;
    std::cout << "====================================================" << std::endl;

    // =========================================================================
    // GUI VISUALIZATION (OPTIONAL)
    // =========================================================================

    // Launch the window main loop to visually inspect the final generated mesh 
    // (typically TERNARY_KRF at the maximum resolution grid of 120)
    app.Run();

    return 0;
}
*/



/*

// =========================================================================
// Main program for ternary or N-ary Rfunction evaluation and rendering
// =========================================================================


int main(int argc, char** argv) {

    // =========================================================================
    // INITIALIZATION & PARAMETERS CONFIGURATION
    // 
    // Select app behavior through app.currentMode, see below
    // 
    // =========================================================================

    // unique instance of controller application that accesses everything
    AppController app;

    //current Mode controls how the program reacts (keybord, rendering, etc.)
    app.currentMode = AppController::SceneMode::ClassicalExample;
        
        //AppController::SceneMode::ClassicalExample; // 3 spheres or regular polygon case
        //AppController::SceneMode::NaryExample; // used for daisy rfunction rendering over a quad
        //AppController::SceneMode::NaryConvexityTests;  // same with more variable parameters used for rfunction rendering over a quad
        //AppController::SceneMode::CADjunction; // cannot be used as is ==> run dedicated main program to call MC and proper rendering
    
        
    //OpenGL initialisation + implicit scene construction
    app.Init(argc, argv);

    //grid evaluation + openglList
    app.ComputeAndBufferMesh(500);

    // =========================================================================
    // GUI VISUALIZATION (OPTIONAL)
    // =========================================================================

    // Launch the window main loop to visually inspect the final generated colored surface
    // 
    // shortcuts : + and - ==> zoom in/out
    // 0 to 7 : render various implicit functions
    // i : switch to intersection to union
    // p : switch to flat or 3D rendering
    // r : reset rotation
    // enter : close app

    app.Run();

    return 0;
}

*/