#include "AppConfig.h"

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <unordered_map>

// ============================================================
// Small parsing helpers
// ============================================================

namespace {

bool Has(const YAML::Node& node, const std::string& key)
{
    return node && node[key];
}

template <typename T>
T ReadOr(const YAML::Node& node, const std::string& key, const T& defaultValue)
{
    if (!Has(node, key)) {
        return defaultValue;
    }

    return node[key].as<T>();
}

Eigen::Vector3d ReadVector3(
    const YAML::Node& node,
    const Eigen::Vector3d& defaultValue
)
{
    if (!node || !node.IsSequence() || node.size() != 3) {
        return defaultValue;
    }

    return Eigen::Vector3d(
        node[0].as<double>(),
        node[1].as<double>(),
        node[2].as<double>()
    );
}

LogicType ParseLogicType(const std::string& value)
{
    if (value == "MIN_NAIVE") {
        return LogicType::MIN_NAIVE;
    }

    if (value == "BINARY_RP") {
        return LogicType::BINARY_RP;
    }

    if (value == "TERNARY_KRF") {
        return LogicType::TERNARY_KRF;
    }

    throw std::runtime_error("Unknown LogicType: " + value);
}

JunctionType ParseJunctionType(const std::string& value)
{
    if (value == "X_JUNCTION") {
        return JunctionType::X_JUNCTION;
    }

    if (value == "Y_JUNCTION") {
        return JunctionType::Y_JUNCTION;
    }

    throw std::runtime_error("Unknown JunctionType: " + value);
}

bool ParseLogicalOperation(const std::string& value)
{
    if (value == "intersection") {
        return true;
    }

    if (value == "union") {
        return false;
    }

    throw std::runtime_error(
        "Unknown logical_operation: " + value +
        ". Expected 'union' or 'intersection'."
    );
}



AppController::SceneMode ParseSceneMode(const std::string& value)
{
    if (value == "ClassicalExample") {
        return AppController::SceneMode::ClassicalExample;
    }

    if (value == "NaryExample") {
        return AppController::SceneMode::NaryExample;
    }

    if (value == "BenchmarkGradient") {
        return AppController::SceneMode::BenchmarkGradient;
    }

    if (value == "BenchmarkGradientUnbalancedTree") {
        return AppController::SceneMode::BenchmarkGradientUnbalancedTree;
    }

    if (value == "CADjunction") {
        return AppController::SceneMode::CADjunction;
    }

    throw std::runtime_error("Unknown SceneMode: " + value);
}

AppConfig::FieldSceneType ParseFieldSceneType(const std::string& value)
{
    if (value == "polygon") {
        return AppConfig::FieldSceneType::POLYGON;
    }

    if (value == "spheres") {
        return AppConfig::FieldSceneType::SPHERES;
    }

    if (value == "daisy") {
        return AppConfig::FieldSceneType::DAISY;
    }

    
    throw std::runtime_error("Unknown field scene type: " + value);
}

std::vector<std::string> ReadStringList(const YAML::Node& node)
{
    std::vector<std::string> result;

    if (!node || !node.IsSequence()) {
        return result;
    }

    for (const auto& item : node) {
        result.push_back(item.as<std::string>());
    }

    return result;
}

std::vector<int> ReadIntList(const YAML::Node& node)
{
    std::vector<int> result;

    if (!node || !node.IsSequence()) {
        return result;
    }

    for (const auto& item : node) {
        result.push_back(item.as<int>());
    }

    return result;
}

std::vector<LogicType> ReadLogicList(const YAML::Node& node)
{
    std::vector<LogicType> result;

    if (!node || !node.IsSequence()) {
        return result;
    }

    for (const auto& item : node) {
        result.push_back(ParseLogicType(item.as<std::string>()));
    }

    return result;
}

std::vector<AppConfig::SphereConfig> ReadSpheres(const YAML::Node& node)
{
    std::vector<AppConfig::SphereConfig> spheres;

    if (!node || !node.IsSequence()) {
        return spheres;
    }

    for (const auto& item : node) {
        AppConfig::SphereConfig sphere;

        sphere.center = ReadVector3(item["center"], sphere.center);
        sphere.radius = ReadOr<double>(item, "radius", sphere.radius);

        spheres.push_back(sphere);
    }

    return spheres;
}

AppConfig::FieldSceneConfig ReadFieldScene(const YAML::Node& node)
{
    AppConfig::FieldSceneConfig cfg;

    if (!node) {
        return cfg;
    }

    if (Has(node, "type")) {
        cfg.type = ParseFieldSceneType(node["type"].as<std::string>());
    }

    if (Has(node, "polygon")) {
        const auto polygon = node["polygon"];

        cfg.polygon.nSides =
            ReadOr<int>(polygon, "n_sides", cfg.polygon.nSides);

        cfg.polygon.radius =
            ReadOr<double>(polygon, "radius", cfg.polygon.radius);
    }

    if (Has(node, "spheres")) {
        cfg.spheres = ReadSpheres(node["spheres"]);
    }

    if (Has(node, "daisy")) {
        const auto daisy = node["daisy"];

        cfg.daisy.primitiveRadius =
            ReadOr<double>(daisy, "primitive_radius", cfg.daisy.primitiveRadius);

        cfg.daisy.centerRadius =
            ReadOr<double>(daisy, "center_radius", cfg.daisy.centerRadius);

        cfg.daisy.primitiveCount =
            ReadOr<int>(daisy, "primitive_count", cfg.daisy.primitiveCount);
    }

    return cfg;
}

void ReadMarchingCubes(
    const YAML::Node& node,
    AppConfig::MarchingCubesConfig& cfg
)
{
    if (!node) {
        return;
    }

    cfg.minBound =
        ReadVector3(node["min_bound"], cfg.minBound);

    cfg.maxBound =
        ReadVector3(node["max_bound"], cfg.maxBound);

    cfg.gridResolution =
        ReadOr<int>(node, "grid_resolution", cfg.gridResolution);
}

void ReadCADThresholds(
    const YAML::Node& node,
    AppConfig::CADThresholdsConfig& cfg
)
{
    if (!node) {
        return;
    }

    cfg.initEpsilonFactorDx =
        ReadOr<double>(
            node,
            "init_epsilon_factor_dx",
            cfg.initEpsilonFactorDx
        );

    cfg.areaThresholdFactorInitSquared =
        ReadOr<double>(
            node,
            "area_threshold_factor_init_squared",
            cfg.areaThresholdFactorInitSquared
        );

    cfg.finalEpsilonFactorInit =
        ReadOr<double>(
            node,
            "final_epsilon_factor_init",
            cfg.finalEpsilonFactorInit
        );

    cfg.finalAreaThresholdFactor =
        ReadOr<double>(
            node,
            "final_area_threshold_factor",
            cfg.finalAreaThresholdFactor
        );

    cfg.sharpAngleDeg =
        ReadOr<double>(
            node,
            "sharp_angle_deg",
            cfg.sharpAngleDeg
        );
}

void ReadCADPipeline(
    const YAML::Node& node,
    AppConfig::CADPipelineConfig& cfg
)
{
    if (!node) {
        return;
    }

    cfg.runMarchingCubes =
        ReadOr<bool>(node, "run_marching_cubes", cfg.runMarchingCubes);

    cfg.weldInitialVertices =
        ReadOr<bool>(node, "weld_initial_vertices", cfg.weldInitialVertices);

    cfg.runTopologyCleanup =
        ReadOr<bool>(node, "run_topology_cleanup", cfg.runTopologyCleanup);

    cfg.buildTopologyAfterCleanup =
        ReadOr<bool>(node, "build_topology_after_cleanup", cfg.buildTopologyAfterCleanup);

    cfg.runValenceOptimization =
        ReadOr<bool>(node, "run_valence_optimization", cfg.runValenceOptimization);

    cfg.valenceMaxPasses =
        ReadOr<int>(node, "valence_max_passes", cfg.valenceMaxPasses);

    cfg.runParasiteCollapse =
        ReadOr<bool>(node, "run_parasite_collapse", cfg.runParasiteCollapse);

    cfg.buildTopologyAfterParasiteCollapse =
        ReadOr<bool>(
            node,
            "build_topology_after_parasite_collapse",
            cfg.buildTopologyAfterParasiteCollapse
        );

    cfg.runTriangleImprovement =
        ReadOr<bool>(node, "run_triangle_improvement", cfg.runTriangleImprovement);

    cfg.triangleImprovementIterations =
        ReadOr<int>(
            node,
            "triangle_improvement_iterations",
            cfg.triangleImprovementIterations
        );

    cfg.projectPointsToSurface =
        ReadOr<bool>(node, "project_points_to_surface", cfg.projectPointsToSurface);

    cfg.buildTopologyBeforeFeatureRefinement =
        ReadOr<bool>(
            node,
            "build_topology_before_feature_refinement",
            cfg.buildTopologyBeforeFeatureRefinement
        );

    cfg.refineSharpEdges =
        ReadOr<bool>(node, "refine_sharp_edges", cfg.refineSharpEdges);

    cfg.runSmallTriangleCollapse =
        ReadOr<bool>(
            node,
            "run_small_triangle_collapse",
            cfg.runSmallTriangleCollapse
        );

    cfg.runUnifiedCleanup =
        ReadOr<bool>(node, "run_unified_cleanup", cfg.runUnifiedCleanup);

    cfg.runFeaturePreservingSmooth =
        ReadOr<bool>(
            node,
            "run_feature_preserving_smooth",
            cfg.runFeaturePreservingSmooth
        );

    cfg.featureSmoothIterations =
        ReadOr<int>(
            node,
            "feature_smooth_iterations",
            cfg.featureSmoothIterations
        );

    cfg.runUltimateMeshSanitizer =
        ReadOr<bool>(
            node,
            "run_ultimate_mesh_sanitizer",
            cfg.runUltimateMeshSanitizer
        );

    cfg.sanitizerMinAreaFactor =
        ReadOr<double>(
            node,
            "sanitizer_min_area_factor",
            cfg.sanitizerMinAreaFactor
        );

    cfg.sanitizerMinAspectRatio =
        ReadOr<double>(
            node,
            "sanitizer_min_aspect_ratio",
            cfg.sanitizerMinAspectRatio
        );
}

} // anonymous namespace

// ============================================================
// Constructor
// ============================================================

AppConfig::AppConfig(const std::string& configPath)
{
    loadFromFile(configPath);
}



// ============================================================
// Private loading function
// ============================================================

void AppConfig::loadFromFile(const std::string& configPath)
{
    m_configPath = FindConfigFile(configPath);

    YAML::Node root = YAML::LoadFile(m_configPath.string());

    // ------------------------------------------------------------
    // Common
    // ------------------------------------------------------------

    if (Has(root, "common")) {
        const auto common = root["common"];

        if (Has(common, "window")) {
            const auto window = common["window"];

            m_common.window.width =
                ReadOr<int>(window, "width", m_common.window.width);

            m_common.window.height =
                ReadOr<int>(window, "height", m_common.window.height);
        }

        if (Has(common, "rfunctions")) {
            const auto rfunctions = common["rfunctions"];

            m_common.rfunctions.rfNumber =
                ReadOr<int>(
                    rfunctions,
                    "rf_number",
                    m_common.rfunctions.rfNumber
                );

            m_common.rfunctions.rvachevM =
                ReadOr<int>(
                    rfunctions,
                    "rvachev_m",
                    m_common.rfunctions.rvachevM
                );

            if (Has(rfunctions, "default_logic")) {
                m_common.rfunctions.defaultLogic =
                    ParseLogicType(rfunctions["default_logic"].as<std::string>());
            }
        }

        if (Has(common, "spherical_evaluator")) {
            const auto sphericalEvaluator = common["spherical_evaluator"];

            m_common.sphericalEvaluator.dimension =
                ReadOr<int>(
                    sphericalEvaluator,
                    "dimension",
                    m_common.sphericalEvaluator.dimension
                );
        }
    }

    // ------------------------------------------------------------
    // FIELD_RENDER
    // ------------------------------------------------------------

    if (Has(root, "field_render")) {
        const auto fieldRender = root["field_render"];

        if (Has(fieldRender, "scene_mode")) {
            m_fieldRender.sceneMode =
                ParseSceneMode(fieldRender["scene_mode"].as<std::string>());
        }

        m_fieldRender.sampling =
            ReadOr<int>(
                fieldRender,
                "sampling",
                m_fieldRender.sampling
            );

        m_fieldRender.isIntersection =
            ReadOr<bool>(
                fieldRender,
                "is_intersection",
                m_fieldRender.isIntersection
            );

        m_fieldRender.renderAs3D =
            ReadOr<bool>(
                fieldRender,
                "render_as_3d",
                m_fieldRender.renderAs3D
            );

        if (Has(fieldRender, "domain")) {
            const auto domain = fieldRender["domain"];

            m_fieldRender.domain.U =
                ReadVector3(domain["U"], m_fieldRender.domain.U);

            m_fieldRender.domain.V =
                ReadVector3(domain["V"], m_fieldRender.domain.V);

            m_fieldRender.domain.LC =
                ReadVector3(domain["LC"], m_fieldRender.domain.LC);

            m_fieldRender.domain.scale =
                ReadOr<double>(
                    domain,
                    "scale",
                    m_fieldRender.domain.scale
                );
        }

        if (Has(fieldRender, "scene")) {
            m_fieldRender.scene = ReadFieldScene(fieldRender["scene"]);
        }

        
        if (Has(fieldRender, "gui")) {
            m_fieldRender.guiEnabled =
                ReadOr<bool>(
                    fieldRender["gui"],
                    "enabled",
                    m_fieldRender.guiEnabled
                );
        }
    }

    // ------------------------------------------------------------
    // CAD_JUNCTION_RENDER
    // ------------------------------------------------------------

    if (Has(root, "cad_junction")) {
        const auto cadJunction = root["cad_junction"];

        if (Has(cadJunction, "junction_type")) {
            m_cadJunction.junctionType =
                ParseJunctionType(cadJunction["junction_type"].as<std::string>());
        }

        m_cadJunction.cylinderRadius =
            ReadOr<double>(
                cadJunction,
                "cylinder_radius",
                m_cadJunction.cylinderRadius
            );

        if (Has(cadJunction, "logical_operation")) {
            m_cadJunction.logicalIntersection =
                ParseLogicalOperation(
                    cadJunction["logical_operation"].as<std::string>()
                );
        }

        if (Has(cadJunction, "logic")) {
            m_cadJunction.logic =
                ParseLogicType(cadJunction["logic"].as<std::string>());
        }

        ReadMarchingCubes(
            cadJunction["marching_cubes"],
            m_cadJunction.marchingCubes
        );

        ReadCADThresholds(
            cadJunction["thresholds"],
            m_cadJunction.thresholds
        );

        ReadCADPipeline(
            cadJunction["pipeline"],
            m_cadJunction.pipeline
        );

        if (Has(cadJunction, "render")) {
            const auto render = cadJunction["render"];

            m_cadJunction.render.meshBufferSampling =
                ReadOr<int>(
                    render,
                    "mesh_buffer_sampling",
                    m_cadJunction.render.meshBufferSampling
                );

            m_cadJunction.render.guiEnabled =
                ReadOr<bool>(
                    render,
                    "gui_enabled",
                    m_cadJunction.render.guiEnabled
                );
        }

        if (Has(cadJunction, "output")) {
            const auto output = cadJunction["output"];

            m_cadJunction.output.saveMesh =
                ReadOr<bool>(
                    output,
                    "save_mesh",
                    m_cadJunction.output.saveMesh
                );

            m_cadJunction.output.meshFile =
                ReadOr<std::string>(
                    output,
                    "mesh_file",
                    m_cadJunction.output.meshFile
                );
        }
    }

    // ------------------------------------------------------------
    // CAD_JUNCTION_BENCHMARK
    // ------------------------------------------------------------

    if (Has(root, "cad_junction_benchmark")) {
        const auto benchmark = root["cad_junction_benchmark"];

        if (Has(benchmark, "junction_type")) {
            m_cadJunctionBenchmark.junctionType =
                ParseJunctionType(benchmark["junction_type"].as<std::string>());
        }

        m_cadJunctionBenchmark.cylinderRadius =
            ReadOr<double>(
                benchmark,
                "cylinder_radius",
                m_cadJunctionBenchmark.cylinderRadius
            );

        if (Has(benchmark, "logical_operation")) {
            m_cadJunctionBenchmark.logicalIntersection =
                ParseLogicalOperation(
                    benchmark["logical_operation"].as<std::string>()
                );
        }

        if (Has(benchmark, "logics")) {
            m_cadJunctionBenchmark.logics =
                ReadLogicList(benchmark["logics"]);
        }

        if (Has(benchmark, "resolutions")) {
            m_cadJunctionBenchmark.resolutions =
                ReadIntList(benchmark["resolutions"]);
        }

        ReadMarchingCubes(
            benchmark["marching_cubes"],
            m_cadJunctionBenchmark.marchingCubes
        );

        ReadCADThresholds(
            benchmark["thresholds"],
            m_cadJunctionBenchmark.thresholds
        );

        ReadCADPipeline(
            benchmark["pipeline"],
            m_cadJunctionBenchmark.pipeline
        );

        if (Has(benchmark, "output")) {
            m_cadJunctionBenchmark.csvFile =
                ReadOr<std::string>(
                    benchmark["output"],
                    "csv_file",
                    m_cadJunctionBenchmark.csvFile
                );
        }

        if (Has(benchmark, "gui")) {
            m_cadJunctionBenchmark.guiEnabledAfterBenchmark =
                ReadOr<bool>(
                    benchmark["gui"],
                    "enabled_after_benchmark",
                    m_cadJunctionBenchmark.guiEnabledAfterBenchmark
                );
        }
    }

    // ------------------------------------------------------------
    // GRADIENT_BENCHMARK
    // ------------------------------------------------------------

    if (Has(root, "gradient_benchmark")) {
        const auto gradient = root["gradient_benchmark"];

        m_gradientBenchmark.isIntersection =
            ReadOr<bool>(
                gradient,
                "is_intersection",
                m_gradientBenchmark.isIntersection
            );

        m_gradientBenchmark.pointsPerCircle =
            ReadOr<int>(
                gradient,
                "points_per_circle",
                m_gradientBenchmark.pointsPerCircle
            );

        m_gradientBenchmark.fieldValueStart =
            ReadOr<double>(
                gradient,
                "field_value_start",
                m_gradientBenchmark.fieldValueStart
            );

        m_gradientBenchmark.fieldValueEnd =
            ReadOr<double>(
                gradient,
                "field_value_end",
                m_gradientBenchmark.fieldValueEnd
            );

        m_gradientBenchmark.numberOfRuns =
            ReadOr<int>(
                gradient,
                "number_of_runs",
                m_gradientBenchmark.numberOfRuns
            );

        if (Has(gradient, "scene")) {
            m_gradientBenchmark.scene = ReadFieldScene(gradient["scene"]);
        }

        if (Has(gradient, "numerical")) {
            const auto numerical = gradient["numerical"];

            m_gradientBenchmark.finiteDifferenceH =
                ReadOr<double>(
                    numerical,
                    "finite_difference_h",
                    m_gradientBenchmark.finiteDifferenceH
                );

            m_gradientBenchmark.boundaryTolerance =
                ReadOr<double>(
                    numerical,
                    "boundary_tolerance",
                    m_gradientBenchmark.boundaryTolerance
                );
        }

        if (Has(gradient, "output")) {
            m_gradientBenchmark.csvFile =
                ReadOr<std::string>(
                    gradient["output"],
                    "csv_file",
                    m_gradientBenchmark.csvFile
                );
        }
    }

    // ------------------------------------------------------------
    // GRADIENT_UNBALANCED_TREE_BENCHMARK
    // ------------------------------------------------------------

    if (Has(root, "gradient_unbalanced_tree_benchmark")) {
        const auto gradient = root["gradient_unbalanced_tree_benchmark"];

        m_gradientUnbalancedTreeBenchmark.isIntersection =
            ReadOr<bool>(
                gradient,
                "is_intersection",
                m_gradientUnbalancedTreeBenchmark.isIntersection
            );

        m_gradientUnbalancedTreeBenchmark.pointsPerCircle =
            ReadOr<int>(
                gradient,
                "points_per_circle",
                m_gradientUnbalancedTreeBenchmark.pointsPerCircle
            );

        m_gradientUnbalancedTreeBenchmark.fieldValueStart =
            ReadOr<double>(
                gradient,
                "field_value_start",
                m_gradientUnbalancedTreeBenchmark.fieldValueStart
            );

        m_gradientUnbalancedTreeBenchmark.fieldValueEnd =
            ReadOr<double>(
                gradient,
                "field_value_end",
                m_gradientUnbalancedTreeBenchmark.fieldValueEnd
            );

        m_gradientUnbalancedTreeBenchmark.numberOfRuns =
            ReadOr<int>(
                gradient,
                "number_of_runs",
                m_gradientUnbalancedTreeBenchmark.numberOfRuns
            );

        if (Has(gradient, "scene")) {
            const auto scene = gradient["scene"];

            m_gradientUnbalancedTreeBenchmark.daisy.primitiveRadius =
                ReadOr<double>(
                    scene,
                    "primitive_radius",
                    m_gradientUnbalancedTreeBenchmark.daisy.primitiveRadius
                );

            m_gradientUnbalancedTreeBenchmark.daisy.centerRadius =
                ReadOr<double>(
                    scene,
                    "center_radius",
                    m_gradientUnbalancedTreeBenchmark.daisy.centerRadius
                );

            m_gradientUnbalancedTreeBenchmark.daisy.primitiveCount =
                ReadOr<int>(
                    scene,
                    "primitive_count",
                    m_gradientUnbalancedTreeBenchmark.daisy.primitiveCount
                );
        }

        if (Has(gradient, "numerical")) {
            const auto numerical = gradient["numerical"];

            m_gradientUnbalancedTreeBenchmark.finiteDifferenceH =
                ReadOr<double>(
                    numerical,
                    "finite_difference_h",
                    m_gradientUnbalancedTreeBenchmark.finiteDifferenceH
                );

            m_gradientUnbalancedTreeBenchmark.boundaryTolerance =
                ReadOr<double>(
                    numerical,
                    "boundary_tolerance",
                    m_gradientUnbalancedTreeBenchmark.boundaryTolerance
                );
        }

        if (Has(gradient, "output")) {
            m_gradientUnbalancedTreeBenchmark.csvFile =
                ReadOr<std::string>(
                    gradient["output"],
                    "csv_file",
                    m_gradientUnbalancedTreeBenchmark.csvFile
                );
        }
    }

    // ------------------------------------------------------------
    // NEWTON_RAPHSON_BENCHMARK
    // ------------------------------------------------------------

    if (Has(root, "newton_raphson_benchmark")) {
        const auto newton = root["newton_raphson_benchmark"];

        m_newtonRaphsonBenchmark.gridResolution =
            ReadOr<int>(
                newton,
                "grid_resolution",
                m_newtonRaphsonBenchmark.gridResolution
            );

        if (Has(newton, "domain")) {
            const auto domain = newton["domain"];

            m_newtonRaphsonBenchmark.minBound =
                ReadVector3(
                    domain["min_bound"],
                    m_newtonRaphsonBenchmark.minBound
                );

            m_newtonRaphsonBenchmark.maxBound =
                ReadVector3(
                    domain["max_bound"],
                    m_newtonRaphsonBenchmark.maxBound
                );
        }

        if (Has(newton, "scene")) {
            m_newtonRaphsonBenchmark.scene =
                ReadFieldScene(newton["scene"]);
        }

        if (Has(newton, "numerical")) {
            const auto numerical = newton["numerical"];

            m_newtonRaphsonBenchmark.epsilon =
                ReadOr<double>(
                    numerical,
                    "epsilon",
                    m_newtonRaphsonBenchmark.epsilon
                );

            m_newtonRaphsonBenchmark.maxIterations =
                ReadOr<int>(
                    numerical,
                    "max_iterations",
                    m_newtonRaphsonBenchmark.maxIterations
                );

            m_newtonRaphsonBenchmark.finiteDifferenceH =
                ReadOr<double>(
                    numerical,
                    "finite_difference_h",
                    m_newtonRaphsonBenchmark.finiteDifferenceH
                );

            m_newtonRaphsonBenchmark.sutureThreshold =
                ReadOr<double>(
                    numerical,
                    "suture_threshold",
                    m_newtonRaphsonBenchmark.sutureThreshold
                );
        }

        if (Has(newton, "methods")) {
            m_newtonRaphsonBenchmark.methods =
                ReadStringList(newton["methods"]);
        }

        if (Has(newton, "output")) {
            m_newtonRaphsonBenchmark.csvFile =
                ReadOr<std::string>(
                    newton["output"],
                    "csv_file",
                    m_newtonRaphsonBenchmark.csvFile
                );
        }
    }

    // ------------------------------------------------------------
    // RFUNCTION_PERFORMANCE_BENCHMARK
    // ------------------------------------------------------------

    if (Has(root, "rfunction_performance_benchmark")) {
        const auto benchmark = root["rfunction_performance_benchmark"];

        m_rfunctionPerformanceBenchmark.dimension =
            ReadOr<int>(
                benchmark,
                "dimension",
                m_rfunctionPerformanceBenchmark.dimension
            );

        m_rfunctionPerformanceBenchmark.pointNumber =
            ReadOr<int>(
                benchmark,
                "point_number",
                m_rfunctionPerformanceBenchmark.pointNumber
            );

        m_rfunctionPerformanceBenchmark.isIntersection =
            ReadOr<bool>(
                benchmark,
                "is_intersection",
                m_rfunctionPerformanceBenchmark.isIntersection
            );

        if (Has(benchmark, "methods")) {
            m_rfunctionPerformanceBenchmark.methods =
                ReadStringList(benchmark["methods"]);
        }

        if (Has(benchmark, "output")) {
            const auto output = benchmark["output"];

            m_rfunctionPerformanceBenchmark.printToConsole =
                ReadOr<bool>(
                    output,
                    "print_to_console",
                    m_rfunctionPerformanceBenchmark.printToConsole
                );

            if (Has(output, "csv_file") && !output["csv_file"].IsNull()) {
                m_rfunctionPerformanceBenchmark.csvFile =
                    output["csv_file"].as<std::string>();
            }
        }
    }
}

// ============================================================
// Search config file
// ============================================================

std::filesystem::path AppConfig::FindConfigFile(
    const std::string& requestedPath
)
{
    namespace fs = std::filesystem;

    if (requestedPath.empty()) {
        throw std::runtime_error(
            "No YAML configuration file was provided.\n"
            "Usage:\n"
            "  ./TernaryRFunctions <APPLICATION_MODE> <CONFIG_FILE>\n"
        );
    }

    fs::path path(requestedPath);

    std::error_code ec;

    if (!fs::exists(path, ec)) {
        throw std::runtime_error(
            "YAML configuration file does not exist: " + path.string()
        );
    }

    if (!fs::is_regular_file(path, ec)) {
        throw std::runtime_error(
            "YAML configuration path is not a regular file: " + path.string()
        );
    }

    return fs::absolute(path);
}

// ============================================================
// Application mode parser
// ============================================================

ApplicationMode AppConfig::ParseApplicationMode(const std::string& value)
{
    if (value == "FIELD_RENDER") {
        return ApplicationMode::FIELD_RENDER;
    }

    if (value == "CAD_JUNCTION_RENDER") {
        return ApplicationMode::CAD_JUNCTION_RENDER;
    }

    if (value == "CAD_JUNCTION_BENCHMARK") {
        return ApplicationMode::CAD_JUNCTION_BENCHMARK;
    }

    if (value == "GRADIENT_BENCHMARK") {
        return ApplicationMode::GRADIENT_BENCHMARK;
    }

    if (value == "GRADIENT_UNBALANCED_TREE_BENCHMARK") {
        return ApplicationMode::GRADIENT_UNBALANCED_TREE_BENCHMARK;
    }

    if (value == "NEWTON_RAPHSON_BENCHMARK") {
        return ApplicationMode::NEWTON_RAPHSON_BENCHMARK;
    }

    if (value == "RFUNCTION_PERFORMANCE_BENCHMARK") {
        return ApplicationMode::RFUNCTION_PERFORMANCE_BENCHMARK;
    }

    throw std::runtime_error(
        "Unknown application mode: " + value + "\n"
        "Expected one of:\n"
        "  FIELD_RENDER\n"
        "  CAD_JUNCTION_RENDER\n"
        "  CAD_JUNCTION_BENCHMARK\n"
        "  GRADIENT_BENCHMARK\n"
        "  GRADIENT_UNBALANCED_TREE_BENCHMARK\n"
        "  NEWTON_RAPHSON_BENCHMARK\n"
        "  RFUNCTION_PERFORMANCE_BENCHMARK\n"
    );
}

std::string AppConfig::ToString(ApplicationMode mode)
{
    switch (mode) {
    case ApplicationMode::FIELD_RENDER:
        return "FIELD_RENDER";

    case ApplicationMode::CAD_JUNCTION_RENDER:
        return "CAD_JUNCTION_RENDER";

    case ApplicationMode::CAD_JUNCTION_BENCHMARK:
        return "CAD_JUNCTION_BENCHMARK";

    case ApplicationMode::GRADIENT_BENCHMARK:
        return "GRADIENT_BENCHMARK";

    case ApplicationMode::GRADIENT_UNBALANCED_TREE_BENCHMARK:
        return "GRADIENT_UNBALANCED_TREE_BENCHMARK";

    case ApplicationMode::NEWTON_RAPHSON_BENCHMARK:
        return "NEWTON_RAPHSON_BENCHMARK";

    case ApplicationMode::RFUNCTION_PERFORMANCE_BENCHMARK:
        return "RFUNCTION_PERFORMANCE_BENCHMARK";
    }

    return "UNKNOWN";
}
