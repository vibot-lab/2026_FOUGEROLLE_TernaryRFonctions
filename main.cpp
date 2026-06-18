/**
 * @file main.cpp
 * @author Yohan FOUGEROLLE - Joaquin Rodriguez
 * @date 2026
 * @brief Main entry for several usages of Rfunctions
 */

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>

#include "AppController.h"
#include "AppConfig.h"
#include "ImplicitObjects.h"

// ============================================================
// Console helpers
// ============================================================
void PrintUsage(const char* executableName)
{
    std::cerr
        << "Usage:\n"
        << "  " << executableName << " <APPLICATION_MODE> <CONFIG_FILE>\n\n"
        << "Available APPLICATION_MODE values:\n"
        << "  FIELD_RENDER\n"
        << "  CAD_JUNCTION_RENDER\n"
        << "  CAD_JUNCTION_BENCHMARK\n"
        << "  GRADIENT_BENCHMARK\n"
        << "  GRADIENT_UNBALANCED_TREE_BENCHMARK\n"
        << "  NEWTON_RAPHSON_BENCHMARK\n"
        << "  RFUNCTION_PERFORMANCE_BENCHMARK\n\n"
        << "Examples:\n"
        << "  " << executableName << " FIELD_RENDER config.yaml\n"
        << "  " << executableName << " CAD_JUNCTION_RENDER config.yaml\n"
        << "  " << executableName << " CAD_JUNCTION_BENCHMARK config.yaml\n";
}

void PrintInteractiveControls(ApplicationMode mode)
{
    std::cout
        << "\nInteractive controls:\n"
        << "  Left mouse drag : rotate scene\n"
        << "  +               : zoom in\n"
        << "  -               : zoom out\n"
        << "  R               : reset rotation and view translation\n"
        << "  W               : wireframe mode\n"
        << "  F               : filled surface mode\n"
        << "  ESC             : exit application\n";

    if (mode == ApplicationMode::FIELD_RENDER) {
        std::cout
            << "\nFIELD_RENDER-specific controls:\n"
            << "  0: Min/Max\n"
            << "  1: Chained Rp\n"
            << "  2: Chained R0m\n"
            << "  3: Zenkin Nd\n"
            << "  4: Rvachev Nd\n"
            << "  5: Spherical directional\n"
            << "  6: Spherical normalized\n"
            << "  7 and above: unused \n"
            << "  I: toggle intersection / union\n"
            << "  P: toggle 2D / 3D height-field rendering\n";
    }
    std::cout << std::endl;
}


// ============================================================
// Common setup
// ============================================================

void ApplyCommonConfig(AppController& app, const AppConfig& config)
{
    const auto& common = config.common();

    app.RFnumber = common.rfunctions.rfNumber;
    app.Rvachev_m = common.rfunctions.rvachevM;

    app.SpherEval.Init(
        static_cast<unsigned int>(common.sphericalEvaluator.dimension)
    );

    app.m_scene.window_width = common.window.width;
    app.m_scene.window_height = common.window.height;

    app.Mesh.TernarySphericalOperator = &(app.SpherEval);
    app.m_scene.m_CadLogic->TernarySphericalOperator = &(app.SpherEval);
}


// ============================================================
// Scene construction helpers
// ============================================================

std::vector<std::shared_ptr<ImplicitObject>>
BuildSphereObjects(const std::vector<AppConfig::SphereConfig>& spheres)
{
    std::vector<std::shared_ptr<ImplicitObject>> objects;

    objects.reserve(spheres.size());

    for (const auto& s : spheres) {
        objects.push_back(
            std::make_shared<ImplicitSphere>(s.center, s.radius)
        );
    }

    return objects;
}

void BuildFieldSceneFromConfig(AppController& app,
    const AppConfig::FieldSceneConfig& sceneConfig)
{
    app.m_objects.clear();

    switch (sceneConfig.type) {
    case AppConfig::FieldSceneType::POLYGON:
        app.makePolygon(
            sceneConfig.polygon.nSides,
            sceneConfig.polygon.radius
        );
        break;

    case AppConfig::FieldSceneType::SPHERES:
        app.m_objects = BuildSphereObjects(sceneConfig.spheres);
        break;

    case AppConfig::FieldSceneType::DAISY:
        app.m_objects = app.MakeDaisy(
            sceneConfig.daisy.primitiveRadius,
            sceneConfig.daisy.centerRadius,
            sceneConfig.daisy.primitiveCount
        );
        break;

    
    default:
        throw std::runtime_error("Unsupported field scene type.");
    }
}

// ============================================================
// Pipeline helper
// ============================================================
struct CADDerivedThresholds {
    double dx = 0.0;
    double initEpsilon = 0.0;
    double areaThreshold = 0.0;
    double finalEpsilon = 0.0;
    double finalAreaThreshold = 0.0;
};

CADDerivedThresholds ComputeCADThresholds(
    const AppConfig::MarchingCubesConfig& mc,
    const AppConfig::CADThresholdsConfig& th
)
{
    CADDerivedThresholds out;

    out.dx =
        (mc.maxBound.x() - mc.minBound.x()) /
        static_cast<double>(mc.gridResolution);

    out.initEpsilon =
        out.dx * th.initEpsilonFactorDx;

    out.areaThreshold =
        out.initEpsilon *
        out.initEpsilon *
        th.areaThresholdFactorInitSquared;

    out.finalEpsilon =
        out.initEpsilon * th.finalEpsilonFactorInit;

    out.finalAreaThreshold =
        out.finalEpsilon *
        out.finalEpsilon *
        th.finalAreaThresholdFactor;

    return out;
}

void SetupCADJunctionScene(AppController& app, JunctionType junctionType,
    double cylinderRadius, bool logicalIntersection, LogicType logic)
{
    app.Mesh.ClearAll();

    app.Mesh.TernarySphericalOperator = &(app.SpherEval);
    app.m_scene.m_CadLogic->TernarySphericalOperator = &(app.SpherEval);

    app.m_scene.m_CadLogic->setLogicalOperation(logicalIntersection);

    app.m_objects =
        app.MakeTripleJunctionScene(junctionType, cylinderRadius);

    app.attachPrimitives();
    app.setLogic(logic);
}

void RunCADPipeline(AppController& app, const AppConfig::MarchingCubesConfig& mc,
    const AppConfig::CADThresholdsConfig& thresholds,
    const AppConfig::CADPipelineConfig& pipeline)
{
    const CADDerivedThresholds derived =
        ComputeCADThresholds(mc, thresholds);

    std::cout << "[CAD] dx = " << derived.dx << "\n"
              << "[CAD] initEpsilon = " << derived.initEpsilon << "\n"
              << "[CAD] finalEpsilon = " << derived.finalEpsilon << "\n"
              << "[CAD] finalAreaThreshold = "
              << derived.finalAreaThreshold << "\n";

    // ------------------------------------------------------------
    // Phase 1: isosurface generation and initial cleanup
    // ------------------------------------------------------------

    if (pipeline.runMarchingCubes) {
        std::cout << "\n[PHASE 1] Running Marching Cubes..." << std::endl;

        app.RunMarchingCube(
            mc.minBound,
            mc.maxBound,
            mc.gridResolution
        );
    }

    if (pipeline.weldInitialVertices) {
        std::cout << "[PHASE 1] Welding initial vertices..." << std::endl;
        app.Mesh.WeldVertices(derived.initEpsilon);
    }

    if (pipeline.runTopologyCleanup) {
        std::cout << "[PHASE 1] Running topology cleanup..." << std::endl;
        app.RunTopologyCleanup(derived.initEpsilon);
    }

    if (pipeline.buildTopologyAfterCleanup) {
        std::cout << "[PHASE 1] Building mesh topology..." << std::endl;
        app.Mesh.buildMeshTopology();
    }

    // ------------------------------------------------------------
    // Phase 2: topological optimization
    // ------------------------------------------------------------

    if (pipeline.runValenceOptimization) {
        std::cout << "\n[PHASE 2] Running valence optimization..." << std::endl;

        const int totalFlips =
            app.RunValenceOptimization(pipeline.valenceMaxPasses);

        std::cout << "[PHASE 2] Total edge flips: "
                  << totalFlips << std::endl;
    }

    if (pipeline.runParasiteCollapse) {
        std::cout << "[PHASE 2] Collapsing parasitic structures..." << std::endl;
        app.RunParasiteCollapse(derived.initEpsilon);
    }

    if (pipeline.buildTopologyAfterParasiteCollapse) {
        std::cout << "[PHASE 2] Rebuilding topology..." << std::endl;
        app.Mesh.buildMeshTopology();
    }

    // ------------------------------------------------------------
    // Phase 3: geometric relaxation
    // ------------------------------------------------------------

    if (pipeline.runTriangleImprovement) {
        std::cout << "\n[PHASE 3] Running triangle improvement..." << std::endl;
        app.RunTriangleImprovement(pipeline.triangleImprovementIterations);
    }

    if (pipeline.projectPointsToSurface) {
        std::cout << "[PHASE 3] Projecting points to implicit surface..." << std::endl;
        app.AdjustPoints(*(app.m_scene.m_CadLogic), app.Mesh.vertices);
    }

    if (pipeline.buildTopologyBeforeFeatureRefinement) {
        std::cout << "[PHASE 3] Rebuilding topology before feature refinement..." << std::endl;
        app.Mesh.buildMeshTopology();
    }

    // ------------------------------------------------------------
    // Phase 4: sharp-feature refinement
    // ------------------------------------------------------------

    if (pipeline.refineSharpEdges) {
        std::cout << "\n[PHASE 4] Refining sharp edges..." << std::endl;

        app.m_scene.m_MCGenerator->RefineSharpEdges(
            *(app.m_scene.m_CadLogic),
            app.Mesh,
            thresholds.sharpAngleDeg
        );
    }

    // ------------------------------------------------------------
    // Phase 5: final polishing
    // ------------------------------------------------------------

    if (pipeline.runSmallTriangleCollapse) {
        std::cout << "\n[PHASE 5] Collapsing small triangles..." << std::endl;
        app.RunSmallTriangleCollapse(derived.finalAreaThreshold);
    }

    if (pipeline.runUnifiedCleanup) {
        std::cout << "[PHASE 5] Running unified cleanup..." << std::endl;
        app.RunUnifiedCleanup(derived.finalEpsilon);
    }

    if (pipeline.runFeaturePreservingSmooth) {
        std::cout << "[PHASE 5] Running feature-preserving smoothing..." << std::endl;

        app.RunFeaturePreservingSmooth(
            thresholds.sharpAngleDeg,
            pipeline.featureSmoothIterations
        );
    }

    if (pipeline.runUltimateMeshSanitizer) {
        std::cout << "[PHASE 5] Running ultimate mesh sanitizer..." << std::endl;

        app.RunUltimateMeshSanitizer(
            derived.finalAreaThreshold * pipeline.sanitizerMinAreaFactor,
            pipeline.sanitizerMinAspectRatio
        );
    }

    std::cout << "\n[CAD] Final topology rebuild..." << std::endl;
    app.Mesh.buildMeshTopology();

    std::cout << "[CAD] Final mesh: "
              << app.Mesh.vertices.size() << " vertices, "
              << app.Mesh.faces.size() << " triangles."
              << std::endl;
}


// ============================================================
// Application runners
// ============================================================
int RunFieldRender(AppController& app, const AppConfig& config,
    int& argc, char** argv)
{
    const auto& cfg = config.fieldRender();

    app.currentMode = cfg.sceneMode;
    app.set_isInter(cfg.isIntersection);
    app.m_is3D = cfg.renderAs3D;
    app.m_sampling = cfg.sampling;

    app.m_scene.m_domain.U = cfg.domain.U;
    app.m_scene.m_domain.V = cfg.domain.V;
    app.m_scene.m_domain.LC = cfg.domain.LC;
    app.m_scene.m_domain.scale = cfg.domain.scale;

    if (!cfg.guiEnabled) {
        std::cerr
            << "[FIELD_RENDER] gui.enabled is false. "
            << "This mode currently requires an OpenGL context because "
            << "ComputeAndBufferMesh() creates OpenGL display lists."
            << std::endl;
        return 1;
    }

    app.currentMode = AppController::SceneMode::ClassicalExample;

    app.Init(argc, argv);
    

    // Override AppController::Init() hardcoded scene with config-driven scene.
    BuildFieldSceneFromConfig(app, cfg.scene);

    PrintInteractiveControls(ApplicationMode::FIELD_RENDER);

    app.ComputeAndBufferMesh(cfg.sampling);
    app.updateActiveListIndex();
    app.Run();

    return 0;
}

int RunCADJunctionRender(AppController& app, const AppConfig& config,
    int& argc, char** argv)
{
    const auto& cfg = config.cadJunction();

    app.currentMode = AppController::SceneMode::CADjunction;

    SetupCADJunctionScene(
        app,
        cfg.junctionType,
        cfg.cylinderRadius,
        cfg.logicalIntersection,
        cfg.logic
    );

    if (cfg.render.guiEnabled) {
        app.Init(argc, argv);
    }

    RunCADPipeline(
        app,
        cfg.marchingCubes,
        cfg.thresholds,
        cfg.pipeline
    );

    if (cfg.output.saveMesh) {
        std::cout << "[OUTPUT] Saving mesh to: "
                  << cfg.output.meshFile << std::endl;

        if (!app.Mesh.WriteFile(cfg.output.meshFile)) {
            std::cerr << "[WARNING] Could not write mesh file: "
                      << cfg.output.meshFile << std::endl;
        }
    }

    if (cfg.render.guiEnabled) {
        std::cout << "[RENDER] Buffering mesh..." << std::endl;
        app.ComputeAndBufferMesh(cfg.render.meshBufferSampling);
        app.updateActiveListIndex();
        PrintInteractiveControls(ApplicationMode::CAD_JUNCTION_RENDER);
        app.Run();
    }

    return 0;
}

int RunCADJunctionBenchmark(AppController& app, const AppConfig& config,
    int& argc, char** argv)
{
    const auto& cfg = config.cadJunctionBenchmark();

    app.currentMode = AppController::SceneMode::CADjunction;

    SetupCADJunctionScene(
        app,
        cfg.junctionType,
        cfg.cylinderRadius,
        cfg.logicalIntersection,
        LogicType::TERNARY_KRF
    );

    /*
     * IMPORTANT:
     * AppController::RunFullBenchmark() still uses hardcoded:
     *   - logics
     *   - resolutions
     *   - bounds
     *   - thresholds
     *   - output file name
     *
     * So this call executes the existing benchmark behavior.
     */
    if (cfg.guiEnabledAfterBenchmark) {
        app.Init(argc, argv);
    }

    app.RunFullBenchmark();

    if (cfg.guiEnabledAfterBenchmark) {
        app.ComputeAndBufferMesh(100);
        app.updateActiveListIndex();
        app.Run();
    }

    return 0;
}

int RunGradientBenchmark(AppController& app, const AppConfig& config)
{
    const auto& cfg = config.gradientBenchmark();

    app.currentMode = AppController::SceneMode::BenchmarkGradient;

    /*
     * TODO: The current BenchmarkGradient() implementation internally builds
     * its own sphere setup. The scene block is already parsed in AppConfig,
     * but not yet consumed.
     */
    app.BenchmarkGradient(
        cfg.isIntersection,
        cfg.pointsPerCircle,
        cfg.fieldValueStart,
        cfg.fieldValueEnd,
        cfg.numberOfRuns
    );

    return 0;
}

int RunGradientUnbalancedTreeBenchmark(AppController& app, const AppConfig& config)
{
    const auto& cfg = config.gradientUnbalancedTreeBenchmark();

    app.currentMode =
        AppController::SceneMode::BenchmarkGradientUnbalancedTree;

    /*
     * TODO: The current BenchmarkGradientUnbalancedTree() implementation controls
     * its own object generation. The parsed daisy parameters are ready in
     * AppConfig but require a small AppController refactor to be consumed.
     */
    app.BenchmarkGradientUnbalancedTree(
        cfg.isIntersection,
        cfg.pointsPerCircle,
        cfg.fieldValueStart,
        cfg.fieldValueEnd,
        cfg.numberOfRuns
    );

    return 0;
}

int RunNewtonRaphsonBenchmark(AppController& app, const AppConfig& config)
{
    const auto& cfg = config.newtonRaphsonBenchmark();

    /*
     * TODO: Current RunNewtonRaphsonBenchmark() accepts only gridRes.
     * The other parsed parameters are available in AppConfig but require
     * extending AppController::RunNewtonRaphsonBenchmark().
     */

    app.RunNewtonRaphsonBenchmark(cfg.gridResolution);

    return 0;
}

int RunRFunctionPerformanceBenchmark(AppController& app, const AppConfig& config)
{
    const auto& cfg = config.rfunctionPerformanceBenchmark();

    /*
     * TODO: Existing method signature:
     *   ComparisonBenchmark(int dim_start, int point_number)
     *
     * The is_intersection and methods fields are parsed but not consumed
     * by the existing AppController method yet.
     */
    app.ComparisonBenchmark(
        cfg.dimension,
        cfg.pointNumber
    );

    return 0;
}

int main(int argc, char** argv) {
    try {
        /*if (argc < 2) {
            PrintUsage(argv[0]);
            return 1;
        }*/

        //    << "  FIELD_RENDER\n"
        //    << "  CAD_JUNCTION_RENDER\n"
        //    << "  CAD_JUNCTION_BENCHMARK\n"
        //    << "  GRADIENT_BENCHMARK\n"
        //    << "  GRADIENT_UNBALANCED_TREE_BENCHMARK\n"
        //    << "  NEWTON_RAPHSON_BENCHMARK\n"
        //    << "  RFUNCTION_PERFORMANCE_BENCHMARK\n\n"

        // manual parameter settings...
        const char* argv0 = "FIELD_RENDER";
        const char* argv1 = "C:\\Development-C++\\2026_CAD_FOUGEROLLE_TernaryRFonctions_code\\config.yaml";
        const ApplicationMode mode = AppConfig::ParseApplicationMode(argv0);

        const std::string configPath = argv1;

        AppConfig config(configPath);

        std::cout << "[CONFIG] Loaded: " << config.getConfigPath() << std::endl;

        std::cout << "[MODE] " << AppConfig::ToString(mode) << std::endl;

        AppController app;
        ApplyCommonConfig(app, config);

        switch (mode) {
            case ApplicationMode::FIELD_RENDER:
            {                
                return RunFieldRender(app, config, argc, argv);
            }

            case ApplicationMode::CAD_JUNCTION_RENDER:
            {
                return RunCADJunctionRender(app, config, argc, argv);
            }

            case ApplicationMode::CAD_JUNCTION_BENCHMARK:
            {
                return RunCADJunctionBenchmark(app, config, argc, argv);
            }

            case ApplicationMode::GRADIENT_BENCHMARK:
            {
                return RunGradientBenchmark(app, config);
            }

            case ApplicationMode::GRADIENT_UNBALANCED_TREE_BENCHMARK:
            {
                return RunGradientUnbalancedTreeBenchmark(app, config);
            }

            case ApplicationMode::NEWTON_RAPHSON_BENCHMARK:
            {
                return RunNewtonRaphsonBenchmark(app, config);
            }

            case ApplicationMode::RFUNCTION_PERFORMANCE_BENCHMARK:
            {
                return RunRFunctionPerformanceBenchmark(app, config);
            }
        }

        std::cerr << "[ERROR] Unhandled application mode." << std::endl;
        return 1;
    }
    catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << std::endl;
        return 1;
    }
}
