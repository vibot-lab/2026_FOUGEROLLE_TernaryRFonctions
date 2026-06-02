#pragma once

#include <vector>
#include <map>
#include <utility>
#include <Eigen/Dense>
#include "ImplicitObjects.h"

// Forward declaration to prevent circular dependencies
class NeighborMesh;

/**
 * @brief Represents a single voxel cell within the sampling grid.
 */
struct VoxelCell {
    Eigen::Vector3d p[8];   // Spatial coordinates of the 8 grid corners
    double val[8];          // Scalar field evaluation at each corner
};

/**
 * @brief Stores historical structural ancestry for subdivided vertices.
 */
struct SplitInfo {
    size_t midIdx;          // Index of the newly generated midpoint vertex
    size_t v1Idx;           // Index of the first edge endpoint
    size_t v2Idx;           // Index of the second edge endpoint
};

/**
 * @brief Core generator class implementing Isosurface Extraction and local topological refinement.
 */
class MarchingCubes {
private:
    double m_isoValue = 0.0;
    Eigen::Vector3d m_minBound;
    Eigen::Vector3d m_maxBound;
    int m_gridResolution;

public:
    // =========================================================================
    // ISOSURFACE GENERATION & CLEANUP
    // =========================================================================

    /**
     * @brief Extracts the raw watertight mesh from the scalar implicit field.
     */
    std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3i>> Generate(
        const ImplicitObject& obj,
        const Eigen::Vector3d& minBound,
        const Eigen::Vector3d& maxBound,
        int resolution
    );

    /**
     * @brief Welds duplicate vertices using a spatial hashing structure.
     * @note Kept for backward compatibility or isolated generation routines.
     */
    std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3i>> UnifiedClean(
        const std::vector<Eigen::Vector3d>& vertices,
        const std::vector<Eigen::Vector3i>& faces,
        double weldEpsilon
    );

    // =========================================================================
    // TOPOLOGICAL OPERATIONS
    // =========================================================================

    /**
     * @brief Maximizes valence-6 distribution by performing independent set local edge flips.
     */
    int OptimizeValenceFlips(
        const ImplicitObject& obj,
        std::vector<Eigen::Vector3d>& vertices,
        std::vector<Eigen::Vector3i>& faces
    );

    /**
     * @brief Subdivides specified mesh edges and logs ancestry maps to history.
     */
    void RefineTopology(
        NeighborMesh& mesh,
        const std::vector<std::pair<size_t, size_t>>& edgesToSplit,
        std::vector<SplitInfo>& history
    );

    /**
     * @brief Generates a new midpoint vertex on an edge or returns its existing index.
     */
    size_t CreateMidpoint(
        NeighborMesh& mesh,
        size_t v1,
        size_t v2,
        std::map<std::pair<size_t, size_t>, size_t>& splitEdges,
        std::vector<SplitInfo>& history
    );

    /**
     * @brief Query utility to check if an edge has already been subdivided during the current pass.
     */
    
    size_t GetExistingMidpoint(
        size_t v1,
        size_t v2,
        const std::map<std::pair<size_t, size_t>, size_t>& splitEdges
    ) const;

    // =========================================================================
    // GEOMETRIC REFINEMENT & PROJECTION
    // =========================================================================

    /**
     * @brief Linearly interpolates spatial positions based on their field potentials.
     */
    Eigen::Vector3d Interpolate(
        const Eigen::Vector3d& p1,
        const Eigen::Vector3d& p2,
        double v1,
        double v2
    );

    /**
     * @brief Projects an arbitrary point onto the exact implicit isosurface f(P) = 0.
     */
    Eigen::Vector3d ProjectToSurface(
        const ImplicitObject& obj,
        const Eigen::Vector3d& P
    );

    /**
     * @brief Performs dichotomy (bisection) tracking to snap an index perfectly onto the feature boundary.
     */
    void DichotomyRefine(
        const ImplicitObject& obj,
        size_t midIdx,
        size_t v1Idx,
        size_t v2Idx,
        std::vector<Eigen::Vector3d>& vertices,
        int maxIter
    );

    /**
     * @brief Tracks high-curvature ridges and injects sampling density along sharp features.
     */
    void RefineSharpEdges(
        const ImplicitObject& obj,
        NeighborMesh& mesh,
        double angleThresholdDeg
    );
};

