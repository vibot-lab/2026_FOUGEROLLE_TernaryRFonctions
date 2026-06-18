#ifndef __APP_CONFIG_H__
#define __APP_CONFIG_H__

#pragma once

#include <string>
#include <vector>
#include <optional>
#include <stdexcept>
#include <filesystem>

#include <Eigen/Core>

#include "AppController.h"
#include "Rfunctions.h"

enum class ApplicationMode {
    FIELD_RENDER,
    CAD_JUNCTION_RENDER,
    CAD_JUNCTION_BENCHMARK,
    GRADIENT_BENCHMARK,
    GRADIENT_UNBALANCED_TREE_BENCHMARK,
    NEWTON_RAPHSON_BENCHMARK,
    RFUNCTION_PERFORMANCE_BENCHMARK
};

// ============================================================
// Config class
// ============================================================

class AppConfig {
public:

    // ------------------------------------------------------------
    // Basic structs
    // ------------------------------------------------------------

    struct WindowConfig {
        int width = 1280;
        int height = 720;
    };

    struct RFunctionsConfig {
        int rfNumber = 8; //number of Rfunctions to be evaluated
        int rvachevM = 1; // the m parameter in rvachev R0m et chained functions  
        LogicType defaultLogic = LogicType::TERNARY_KRF;
    };

    // ternary operator
    struct SphericalEvaluatorConfig {
        int dimension = 3;
    };

    struct CommonConfig {
        WindowConfig window;
        RFunctionsConfig rfunctions;
        SphericalEvaluatorConfig sphericalEvaluator;
    };

    //default sphere config : unit sphere centered at the origin
    struct SphereConfig {
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        double radius = 1.0;
    };

    //default polygon (always centered at the origin) is hexagon
    struct PolygonConfig {
        int nSides = 6;
        double radius = 1.25;
    };

    // Daisy stands for Daisy flower:
    // The X (usually 9) primitives, usually spheres, generate a potential field that looks like a daisy flower.
    struct DaisyConfig {
        double primitiveRadius = 0.75;
        double centerRadius = 1.0;
        int primitiveCount = 9;
    };


    // Domain definition (left corner + 2 vectors to define a plane + scale)
    // used to evaluate and render Rfunction, usually on plane z=0
    struct FieldDomainConfig {
        Eigen::Vector3d U = Eigen::Vector3d::UnitX();
        Eigen::Vector3d V = Eigen::Vector3d::UnitY();
        Eigen::Vector3d LC = Eigen::Vector3d(-2.0, -2.0, 0.0);
        double scale = 4.0;
    };

    enum class FieldSceneType {
        POLYGON, //used to render regular implicit polygons
        SPHERES, //used to render field at z=0 of spheres (usually 3)
        DAISY    //same as SPHERES with more parameters (number of spheres, radii, etc)
    };

    struct FieldSceneConfig {
        FieldSceneType type = FieldSceneType::DAISY;
        PolygonConfig polygon;
        std::vector<SphereConfig> spheres;
        DaisyConfig daisy;
    };

   

    struct FieldRenderConfig {
        AppController::SceneMode sceneMode = AppController::SceneMode::NaryExample;
        int sampling = 500;
        bool isIntersection = true;
        bool renderAs3D = false;
        FieldDomainConfig domain;
        FieldSceneConfig scene;
        bool guiEnabled = true;
    };


    // Marching cube requires a volume and resolution
    struct MarchingCubesConfig {
        Eigen::Vector3d minBound = Eigen::Vector3d(-1.5, -1.5, -1.5);
        Eigen::Vector3d maxBound = Eigen::Vector3d(1.5, 1.5, 1.5);
        int gridResolution = 80;
    };

    //numerical constants for Mesh update (simplification, cleaning, face collapse...)
    struct CADThresholdsConfig {
        double initEpsilonFactorDx = 0.01;
        double areaThresholdFactorInitSquared = 5.0;
        double finalEpsilonFactorInit = 2.0;
        double finalAreaThresholdFactor = 1.0;
        double sharpAngleDeg = 20.0;
    };


    //this can be adjusted to test and evaluate all steps
    struct CADPipelineConfig {
        bool runMarchingCubes = true;
        bool weldInitialVertices = true;
        bool runTopologyCleanup = true;
        bool buildTopologyAfterCleanup = true;

        bool runValenceOptimization = true;
        int valenceMaxPasses = 5;

        bool runParasiteCollapse = true;
        bool buildTopologyAfterParasiteCollapse = true;

        bool runTriangleImprovement = true;
        int triangleImprovementIterations = 50;

        bool projectPointsToSurface = true;
        bool buildTopologyBeforeFeatureRefinement = true;

        bool refineSharpEdges = true;

        bool runSmallTriangleCollapse = true;
        bool runUnifiedCleanup = true;

        bool runFeaturePreservingSmooth = true;
        int featureSmoothIterations = 10;

        bool runUltimateMeshSanitizer = true;
        double sanitizerMinAreaFactor = 2.0;
        double sanitizerMinAspectRatio = 0.06;
    };

    struct CADRenderConfig {
        int meshBufferSampling = 100;
        bool guiEnabled = true;
    };

    struct CADOutputConfig {
        bool saveMesh = false;
        std::string meshFile = "output.iv";
    };

    struct CADJunctionConfig {
        JunctionType junctionType = JunctionType::X_JUNCTION;
        double cylinderRadius = 0.5;

        // false = union, true = intersection
        bool logicalIntersection = false;

        LogicType logic = LogicType::TERNARY_KRF;

        MarchingCubesConfig marchingCubes;
        CADThresholdsConfig thresholds;
        CADPipelineConfig pipeline;
        CADRenderConfig render;
        CADOutputConfig output;
    };


    // set up the config for the union of 3 cylinders
    // and all the parameters to analyse results
    struct CADJunctionBenchmarkConfig {
        JunctionType junctionType = JunctionType::X_JUNCTION;
        double cylinderRadius = 0.5;

        // false = union, true = intersection
        bool logicalIntersection = false;

        std::vector<LogicType> logics {
            LogicType::MIN_NAIVE,
            LogicType::BINARY_RP,
            LogicType::TERNARY_KRF
        };

        std::vector<int> resolutions {
            50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150
        };

        MarchingCubesConfig marchingCubes;
        CADThresholdsConfig thresholds;
        CADPipelineConfig pipeline;

        std::string csvFile = "benchmark_final.csv";
        bool guiEnabledAfterBenchmark = false;
    };


    // Configuration for Gradient analysis (domain, accuracy, point density, etc)
    struct GradientBenchmarkConfig {
        bool isIntersection = true;
        int pointsPerCircle = 1000;
        double fieldValueStart = -0.5;
        double fieldValueEnd = 0.5;
        int numberOfRuns = 50;

        FieldSceneConfig scene;
        double finiteDifferenceH = 1.0e-4;
        double boundaryTolerance = 1.0e-3;

        std::string csvFile = "BenchmarkGradient.csv";
    };

    //configuration for N-ary unbalanced Rfunction evaluation

    struct GradientUnbalancedTreeBenchmarkConfig {
        bool isIntersection = true;
        int pointsPerCircle = 1000;
        double fieldValueStart = -0.5;
        double fieldValueEnd = 0.5;
        int numberOfRuns = 50;

        DaisyConfig daisy;
        double finiteDifferenceH = 1.0e-4;
        double boundaryTolerance = 1.0e-3;

        std::string csvFile = "BenchmarkGradientUnbalanced.csv";
    };

    // config for NewtonRaphson convergency
    // requires the domain, precision and thresholds, max number of iterations 
    struct NewtonRaphsonBenchmarkConfig {
        int gridResolution = 80;

        Eigen::Vector3d minBound = Eigen::Vector3d(-2.0, -2.0, -2.0);
        Eigen::Vector3d maxBound = Eigen::Vector3d(2.0, 2.0, 2.0);

        FieldSceneConfig scene;

        double epsilon = 1.0e-6;
        int maxIterations = 2000;
        double finiteDifferenceH = 1.0e-6;
        double sutureThreshold = 1.0e-6;

        std::vector<std::string> methods;
        std::string csvFile = "BenchmarkNewtonConvergence.csv";
    };

    //same as above for the performance comparison
    struct RFunctionPerformanceBenchmarkConfig {
        int dimension = 9;
        int pointNumber = 100000;
        bool isIntersection = true;

        std::vector<std::string> methods;
        bool printToConsole = true;
        std::optional<std::string> csvFile = std::nullopt;
    };

public:

    // Constructor loads the YAML immediately.
    // If configPath is empty, it searches for config.yaml/config.yml automatically.
    // Constructor loads the YAML immediately.
    // The config file path is mandatory.
    explicit AppConfig(const std::string& configPath);


    // Application mode parser.
    
    static  ApplicationMode ParseApplicationMode(const std::string& modeStr);
    static std::string ToString(ApplicationMode mode);
    

    // Search utility.
    static std::filesystem::path FindConfigFile(
        const std::string& requestedPath
    );

    // Getters.
    const std::filesystem::path& getConfigPath() const { return m_configPath; }

    const CommonConfig& common() const { return m_common; }
    const FieldRenderConfig& fieldRender() const { return m_fieldRender; }
    const CADJunctionConfig& cadJunction() const { return m_cadJunction; }
    const CADJunctionBenchmarkConfig& cadJunctionBenchmark() const { return m_cadJunctionBenchmark; }
    const GradientBenchmarkConfig& gradientBenchmark() const { return m_gradientBenchmark; }
    const GradientUnbalancedTreeBenchmarkConfig& gradientUnbalancedTreeBenchmark() const { return m_gradientUnbalancedTreeBenchmark; }
    const NewtonRaphsonBenchmarkConfig& newtonRaphsonBenchmark() const { return m_newtonRaphsonBenchmark; }
    const RFunctionPerformanceBenchmarkConfig& rfunctionPerformanceBenchmark() const { return m_rfunctionPerformanceBenchmark; }

private:
    void loadFromFile(const std::string& configPath);

private:

    std::filesystem::path m_configPath;

    CommonConfig m_common;
    FieldRenderConfig m_fieldRender;
    CADJunctionConfig m_cadJunction;
    CADJunctionBenchmarkConfig m_cadJunctionBenchmark;
    GradientBenchmarkConfig m_gradientBenchmark;
    GradientUnbalancedTreeBenchmarkConfig m_gradientUnbalancedTreeBenchmark;
    NewtonRaphsonBenchmarkConfig m_newtonRaphsonBenchmark;
    RFunctionPerformanceBenchmarkConfig m_rfunctionPerformanceBenchmark;
};

#endif // __APP_CONFIG_H__