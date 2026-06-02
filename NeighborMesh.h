/***************************************************************************
 * NeighborMesh.h
 * -------------------
 * Update               : 2026-05-26
 * Original Copyright   : (C) 2002 by Michaël ROY
 * Contact              : michaelroy@users.sourceforge.net
 * * Refactored & Edited  : Yohan Fougerolle
 * Affiliation          : Université Bourgogne Europe / IMVIA UR 7535
 * Warning              : Designed strictly for triangular meshes!
 *
 ***************************************************************************/

 /**
 * @file NeighborMesh.h
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


#ifndef _NEIGHBORMESH_H_
#define _NEIGHBORMESH_H_



#include "Mesh.h"

// NeighborMesh = MESH + Neighborhood
// everything is managed by indexes to avoid tricky pointers issue
// mimics half edge, slightly modified

/**
 * @name Topological Neighborhood Maps
 * @brief Adjacency maps tracking simplicial mesh connectivity parameters.
 * @details Powering the KRF framework's deterministic optimization passes, including
 * valence regularization, edge-collapse operations, and tangential Laplacian smoothing.
 */
 ///@{

 /**
  * @brief P2P_Neigh: Point-to-Point (Vertex-to-Vertex) Neighborhood Graph.
  * @details Stores adjacent vertex indices for each mesh node. If two vertices
  * share a common edge, their indices are added symmetrically to each other's
  * adjacency set to build an undirected connectivity graph.
  * * @code
  * size_t idP = 42; // Target vertex query index
  * * // Access the set of all vertex indices directly connected to idP
  * const std::set<size_t>& adjacent_vertices = P2P_Neigh[idP];
  * * for (size_t neighborId : adjacent_vertices) {
  * // Process localized one-ring neighborhood vertices
  * }
  * @endcode
  */

  /**
   * @brief P2F_Neigh: Point-to-Face (Vertex-to-Face) Adjacency Map.
   * @details Implements inverse incidence tracking by mapping a vertex index onto
   * the collection of all faces that contain it. Essential for computing continuous
   * vertex normals weighted by incident face areas.
   * * @code
   * size_t idP = 42; // Target vertex query index
   * * // Access the set of all face indices sharing vertex idP
   * const std::set<size_t>& incident_faces = P2F_Neigh[idP];
   * * for (size_t faceId : incident_faces) {
   * // Access and process geometric properties of adjacent faces
   * }
   * @endcode
   */

   /**
    * @brief F2F_Neigh: Face-to-Face Adjacency Map.
    * @details Contains the indices of all faces sharing a topological junction
    * (at least one vertex) with the query face. Used for structural region growing
    * or localized normal alignment filters passes.
    * * @code
    * size_t idF = 42; // Target face query index
    * * // Access the set of all face indices adjacent to idF
    * const std::set<size_t>& neighboring_faces = F2F_Neigh[idF];
    * * for (size_t adjFaceId : neighboring_faces) {
    * // Process adjacent macro triangular patches
    * }
    * @endcode
    */

    /**
     * @brief Edges: Canonical Edge-to-Face Connectivity Map.
     * @details Edges connect a pair of vertices and are supported by incident faces.
     * To maintain structural consistency, each edge key is stored as a canonically
     * sorted pair $(v_{min}, v_{max})$, mapped onto a std::set of face indices:
     * - Boundary edges (Open contours): Supported by exactly 1 face.
     * - Manifold internal edges: Supported by exactly 2 faces.
     * - Non-manifold ridge junctions: Supported by 3 or more faces.
     * * @code
     * size_t i1 = 42;
     * size_t i2 = 14;
     * * // Retrieve the continuous map iterator for the edge connecting i1 and i2
     * std::map<std::pair<size_t, size_t>, std::set<size_t>>::const_iterator edgeIt = Get_Edge(i1, i2);
     * * if (edgeIt != Edges.end()) {
     * // 1. Extract the canonically sorted vertex pair from the map key component
     * const std::pair<size_t, size_t>& canonical_pair = edgeIt->first;
     * * // 2. Extract the set of face indices supporting this edge from the map value component
     * const std::set<size_t>& sharing_faces = edgeIt->second;
     * * std::cout << "Edge found connecting sorted nodes: ("
     * << canonical_pair.first << ", " << canonical_pair.second << ")\n";
     * std::cout << "Number of supporting faces: " << sharing_faces.size() << "\n";
     * * // 3. Iterate over the incident faces to evaluate localized edge properties
     * for (size_t faceId : sharing_faces) {
     * // Evaluate normal configurations along the implicit junction ridge line
     * }
     * } else {
     * std::cout << "Topological error: No edge exists between vertex " << i1 << " and " << i2 << "\n";
     * }
     * @endcode
     */
     ///@}

class NeighborMesh : public Mesh {
public:
    // =========================================================================
    // CORE LIFECYCLE & INITIALIZATION
    // =========================================================================
    NeighborMesh();
    virtual ~NeighborMesh();

    /**
     * @brief High-level unifier to sequentially compile adjacency matrices and normals.
     */
    void buildMeshTopology() {
        
        //topological operations
        Build_P2P_Neigh();
        Build_P2F_Neigh();
        Build_F2F_Neigh();
        Build_Edges();

        //recompute normals for rendering
        ComputeFaceNormals();
        ComputeVertexNormals();
    }

    /**
     * @brief Resets all topological entities, structures, and geometric tables.
     */
    void ClearAll();

    // =========================================================================
    // TOPOLOGICAL BUILDING & CLEANING
    // =========================================================================
    bool Build_P2P_Neigh();
    bool Build_P2F_Neigh();
    bool Build_F2F_Neigh();
    bool Build_Edges();

    /**
     * @brief Purges vertices isolated or non-indexed by any face in the system.
     */
    void RemoveUnusedVertices();

    /**
     * @brief Erases redundant co-located points within an epsilon radius.
     */
    void WeldVertices(double epsilon_weld = 0.001);

    /**
     * @brief Direct edge manipulation: updates localized adjacency tables when a face is appended.
     */
    void AddFaceToEdges(size_t fIdx);

    /**
     * @brief Direct edge manipulation: updates localized adjacency tables when a face is deleted.
     */
    void RemoveFaceFromEdges(size_t fIdx);

    
    // Per-vertex scalar buffer used for distance fields or heat mapping
    std::vector<double> Labels;

    // =========================================================================
    // ADJACENCY & ELEMENT ACCESSORS
    // =========================================================================
    std::vector<std::set<size_t>> P2P_Neigh; // Point-to-Point neighborhood map
    std::vector<std::set<size_t>> P2F_Neigh; // Point-to-Face neighborhood map
    std::vector<std::set<size_t>> F2F_Neigh; // Face-to-Face neighborhood map
    std::map<std::pair<size_t, size_t>, std::set<size_t>> Edges; // Edge-to-Faces lookup table

    /**
     * @brief Retrieves an edge iterator from its two endpoint vertex indices.
     */
    std::map<std::pair<size_t, size_t>, std::set<size_t>>::const_iterator Get_Edge(size_t i1, size_t i2) const {
        return Edges.find(std::pair<size_t, size_t>(std::min(i1, i2), std::max(i1, i2)));
    }

    /**
     * @brief Query utility to extract the remaining vertex index of a face given one edge.
     */
    size_t GetOppositeVertex(size_t fIdx, size_t v1, size_t v2) const;

    /**
     * @brief Computes N-ring extended neighborhoods.
     */
    std::set<size_t> GetP2P_Neigh(size_t vertexIndex, size_t nRing) const;
    std::set<size_t> GetF2F_Neigh(size_t faceIndex, size_t nRing) const;

    // =========================================================================
    // GEOMETRIC QUERIES & BOUNDARY EXTRACTION
    // =========================================================================

    /**
     * @brief Checks if a face contains an obtuse internal angle.
     * @return The local vertex index triggering the condition, or string bounds equivalent.
     */
    size_t IsObtuse(size_t face_index) const;

    
    /**
     * @brief Structural re-indexing utility for topological consolidation.
     */
    void ReplaceVertexIndex(int oldIdx, int newIdx);

    // =========================================================================
    // ANALYSIS, PATHFINDING & COLORMAPPING
    // =========================================================================
    void BuildDistanceLabels(size_t seedVertexIdx);
    void DrawDijkstra(size_t startPointIndex, size_t endPointIndex);
    void SetColorsFromLabels();

    
    // =========================================================================
    // OPENGL DEBUG & GEOMETRIC VERIFICATION VISUALIZATION
    // =========================================================================
    void DrawP2P_Neigh(size_t i) const; 
    void DrawP2F_Neigh(size_t i) const;
    void DrawF2F_Neigh(size_t i) const;
    void DrawEdge(size_t i);
    void DrawEdge(const std::map<std::pair<size_t, size_t>, std::set<size_t>>::const_iterator& it) const;
    void DrawBoundaryEdges() const; // very useful to visually check if a mesh has boundary or not
    void DrawPoints(std::set<size_t> vertexIndices) const;
    void DrawFaces(std::set<size_t> faceIndices) const;
    //other functions to illustrate 1-ring neighborhoods...
    void IllustrateEdges(size_t vertexIndex) const;
    void IllustrateP2P_Neigh(size_t vertexIndex) const;
    void IllustrateP2F_Neigh(size_t vertexIndex) const;
    void IllustrateF2F_Neigh(size_t faceIndex) const;
    // ... to n-ring neighborhoods
    void IllustratePointNeighborhoodComputation(size_t p_index, size_t n) const;
    void IllustrateFaceNeighborhoodComputation(size_t f_index, size_t n) const;
};

#endif // _NEIGHBORMESH_H_