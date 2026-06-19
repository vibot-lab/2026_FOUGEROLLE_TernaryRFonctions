/***************************************************************************
                                  NeighborMesh.cpp
                             -------------------
    copyright            : (C) 2010 by yohan FOUGEROLLE
    email                : yohan.fougerolle@u-bourgogne.fr
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include "NeighborMesh.h"
#include <GL/glut.h>
#include <algorithm>
#include <queue> // for dijkstra optimized

using namespace std;
using namespace Eigen;

//constructor and destructor

NeighborMesh :: NeighborMesh(){}
NeighborMesh :: ~NeighborMesh(){}

// construction of the various neighborhoods

bool NeighborMesh :: Build_P2P_Neigh()
{
    
    if ( vertices.empty() || faces.empty() ) return false;

    //first build an array of empty sets
    P2P_Neigh.clear();
    P2P_Neigh.resize( vertices.size() );

    // now span all the faces, and adds the points within the corresponding sets
    for( size_t i=0; i<faces.size(); i++)
    {
        Vector3i F(faces[i]);

        P2P_Neigh[ F[0] ].insert(F[1]);
        P2P_Neigh[ F[0] ].insert(F[2]);

        P2P_Neigh[ F[1] ].insert(F[0]);
        P2P_Neigh[ F[1] ].insert(F[2]);

        P2P_Neigh[ F[2] ].insert(F[0]);
        P2P_Neigh[ F[2] ].insert(F[1]);
    }

    return true;
}


void NeighborMesh :: DrawP2P_Neigh( size_t i ) const
{
    if ( P2P_Neigh.empty() || i >= P2P_Neigh.size() )  return;

    glPointSize(5);

    glDisable(GL_LIGHTING);

    //render the considered point in red
    Vector3d P(vertices[i]); 

    glBegin(GL_POINTS);
    glColor3f(1,0,0);
    glVertex3f(P[0], P[1], P[2]);

    //render the neighbors in green
    glColor3f(0,1,0);
    for ( std::set <size_t> :: const_iterator it = P2P_Neigh[i].begin(); it != P2P_Neigh[i].end(); ++it)
    {
        P = vertices[*it];
        glVertex3f(P[0], P[1], P[2]);
    }
    glEnd();
}


// For each point, stores the set of all the connected faces
bool NeighborMesh :: Build_P2F_Neigh()
{
    //usual sanity checks
    if ( vertices.empty() || faces.empty() ) return false;

    P2F_Neigh.clear();
    P2F_Neigh.resize(vertices.size());

    for (size_t i = 0; i < faces.size(); i++) {
        P2F_Neigh[faces[i].x()].insert(i);
        P2F_Neigh[faces[i].y()].insert(i);
        P2F_Neigh[faces[i].z()].insert(i);
    }
    return true;
}

void NeighborMesh :: DrawP2F_Neigh( size_t i ) const
{
    if ( P2F_Neigh.empty() || i >= P2F_Neigh.size() )  return;

    glPointSize(5);
    glColor3f(1,0,0);
    glDisable(GL_LIGHTING);

    //render the considered point in red
    Vector3d P(vertices[i]);
    glBegin(GL_POINTS);
    glVertex3f(P[0], P[1], P[2]);
    glEnd();

    //render the neighbor faces in dark green
    glColor3f(0,0.5,0);

    set <size_t> myset ( P2F_Neigh[i]);

    glEnable(GL_LIGHTING);
    glBegin(GL_TRIANGLES);

    for ( set <size_t> :: const_iterator it = myset.begin(); it != myset.end(); it++) {

        Draw_Face_Normal(*it);
    }

    glEnd();

}


// construct a set of all the surrounding faces to the current face (each face shares at least one vertex)
bool NeighborMesh :: Build_F2F_Neigh()
{
    if (vertices.empty() || faces.empty() || P2F_Neigh.empty()) return false;

    F2F_Neigh.clear();
    F2F_Neigh.reserve(faces.size());

    for (size_t i = 0; i < faces.size(); i++) {
        // Create a temporary set for the current face
        std::set<size_t> neighbors;
        const Vector3i& F = faces[i];

        // 1. Collect all neighbor indices from vertices
        for (size_t j = 0; j < 3; j++) {
            const auto& p_neighbors = P2F_Neigh[F[j]];
            // Range insertion is more efficient than set_union with inserter
            neighbors.insert(p_neighbors.begin(), p_neighbors.end());
        }

        // 2. Remove the current face index from its own neighborhood
        neighbors.erase(i);

        // 3. Move the set into the F2F structure
        F2F_Neigh.push_back(std::move(neighbors));
    }

    return true;
}

void NeighborMesh :: DrawF2F_Neigh( size_t i ) const
{
    if( F2F_Neigh.empty() || i>F2F_Neigh.size() ) return;

    //draw the considered face in red

    glBegin(GL_TRIANGLES);
    glColor3f(1,0,0);
    Draw_Face_Normal(i);

    //draw neighbor faces in blue
    glColor3f(0,0,1);
    set< size_t > myset ( F2F_Neigh[i] );
    for ( set <size_t> :: const_iterator it = myset.begin(); it != myset.end(); it++)       Draw_Face_Normal(*it);
    glEnd();
}


set<size_t> NeighborMesh :: GetP2P_Neigh( size_t p_index, size_t n) const
{

    set <size_t> previous; previous.insert(p_index);    if ( n == 0 ) return previous;
    set <size_t> new_ring = P2P_Neigh[p_index];         if ( n == 1 ) return new_ring;

    set < size_t > myset;

    for(size_t i = 1; i<n; i++)
    {
        // compute the 1 neighbourhood of the previously computed ring

        myset.clear();

        for ( set <size_t> :: const_iterator it(new_ring.begin()); it != new_ring.end(); it++)
        {
            for (set<size_t> :: const_iterator it2(P2P_Neigh[*it].begin()); it2 != P2P_Neigh[*it].end(); it2++)
            {
                myset.insert(*it2);
            }
        }

        set <size_t> dum; //seems uneasy to remove elements while updating the set at the same time ==> dum set for performing the boolean difference

        //extract previous from my set
        set_difference( myset.begin(), myset.end(),
                        previous.begin(), previous.end(),
                        insert_iterator< set<size_t> >(dum, dum.begin())
                        );

        //previous = myset INTERSECTED with new ring
        previous.clear();
        set_intersection( dum.begin(), dum.end(),
                          new_ring.begin(), new_ring.end(),
                          insert_iterator< set<size_t> >(previous, previous.begin())
                          );

        //new_ring = myset MINUS previous
        new_ring.clear();
        set_difference( dum.begin(), dum.end(),
                        previous.begin(), previous.end(),
                        insert_iterator< set<size_t> >(new_ring, new_ring.begin())
                        );

    }

    return new_ring;
}


/**
 * @brief Computes the exact n-th topological face ring neighborhood around a query face.
 * @details Replaces expensive multi-pass set operations (differences and intersections)
 * with an optimized Breadth-First Search (BFS) graph traversal strategy. Memory performance
 * is dramatically enhanced by utilizing contiguous vectors instead of node-allocated sets.
 * @param f_index The initial root face index from which the neighborhood propagates.
 * @param n The target geodesic layer distance (ring radius).
 * @return std::set<size_t> The set containing all face indices residing exactly at distance n.
 */
std::set<size_t> NeighborMesh::GetF2F_Neigh(size_t f_index, size_t n) const
{
    // Base Case 0: Return an isolated set containing only the root face geometry
    if (n == 0)
    {
        return { f_index };
    }

    // Base Case 1: Directly return the pre-compiled immediate 1-ring neighbors map
    if (n == 1)
    {
        return F2F_Neigh[f_index];
    }

    // -----------------------------------------------------------------
    // OPTIMIZED GRAPH TRAVERSAL LAYERING (BFS)
    // -----------------------------------------------------------------
    // Cache array maintaining visited states and precise ring layers heights.
    // Index mapping: vector position matches face index, value stores its layer distance.
    // Initialized to -1 (unvisited).
    std::vector<int> distance_map(faces.size(), -1);

    // Contiguous workspace vectors representing the current front wave and the next ring step
    std::vector<size_t> current_ring;
    std::vector<size_t> next_ring;

    // Seed the traversal structures using early execution anchors
    distance_map[f_index] = 0;

    const std::set<size_t>& immediate_neighbors = F2F_Neigh[f_index];
    current_ring.reserve(immediate_neighbors.size());

    for (size_t adj_face : immediate_neighbors)
    {
        distance_map[adj_face] = 1;
        current_ring.push_back(adj_face);
    }

    // Sequentially radiate outward until reaching the target layer height context n
    for (size_t layer = 1; layer < n; ++layer)
    {
        next_ring.clear();

        // Parse through the current active wavefront coordinates
        for (size_t current_face : current_ring)
        {
            // Traverse adjacent face listings using the pre-compiled topology structures
            for (size_t neighbor_face : F2F_Neigh[current_face])
            {
                // Visited check: if the neighbor is unvisited, it belongs strictly to the next outer layer
                if (distance_map[neighbor_face] == -1)
                {
                    distance_map[neighbor_face] = static_cast<int>(layer + 1);
                    next_ring.push_back(neighbor_face);
                }
            }
        }

        // Fast swap semantics shifting the frontier outward without memory reallocations
        current_ring = std::move(next_ring);

        // Optimization check: break execution early if the wave front encounters a disconnected mesh boundary
        if (current_ring.empty())
        {
            break;
        }
    }

    // -----------------------------------------------------------------
    // FINAL COLLECTOR EXPORT
    // -----------------------------------------------------------------
    // Convert the optimized linear vector buffer back to a std::set to conform 
    // strictly to the method's legacy return signature without fracturing upstream logic.
    std::set<size_t> result_set(current_ring.begin(), current_ring.end());
    return result_set;
}

void NeighborMesh ::DrawPoints ( set <size_t> s) const
{
    glBegin(GL_POINTS);
    for( set<size_t>::const_iterator it(s.begin()); it != s.end(); it++)
    {
        Vector3d P(vertices[*it]);
        glVertex3f(P[0],P[1],P[2]);
    }
    glEnd();
}

void NeighborMesh :: DrawFaces  ( set <size_t> s) const
{
    for( set<size_t>::const_iterator it(s.begin()); it != s.end(); it++)
    {
        Draw_Face_Normal(*it);
    }
}


// Compute the connectivity between edges and their adjacent faces
bool NeighborMesh::Build_Edges()
{
    // 1. Mandatory cleanup
    Edges.clear();

    if (P2F_Neigh.empty() || faces.empty() || vertices.empty()) return false;

    for (const auto& F : faces) {
        for (size_t j = 0; j < F.size(); j++) { 
            size_t i1 = F[j];
            size_t i2 = F[(j + 1) % F.size()];

            // Ensure consistent edge ordering (lower index first)
            if (i1 > i2) std::swap(i1, i2);
            std::pair<size_t, size_t> myedge(i1, i2);

            // 2. Check existence to avoid redundant intersection calculations
            if (Edges.find(myedge) != Edges.end()) continue;

            // 3. Compute intersection of face neighbors for both vertices
            std::set<size_t> sharedFaces;
            std::set_intersection(
                P2F_Neigh[i1].begin(), P2F_Neigh[i1].end(),
                P2F_Neigh[i2].begin(), P2F_Neigh[i2].end(),
                std::inserter(sharedFaces, sharedFaces.begin())
            );

            // Use emplace and move semantics to store the result efficiently
            Edges.emplace(myedge, std::move(sharedFaces));
        }
    }
    return true;
}

void NeighborMesh :: DrawEdge(const map< pair<size_t, size_t>, set<size_t> > :: const_iterator &it) const
{
    const pair < pair<size_t,size_t>,set<size_t> > myedge = *it;

    Vector3d A(vertices[myedge.first.first]),B(vertices[myedge.first.second]);

    glDisable(GL_LIGHTING);
    glPointSize(5);

    glBegin(GL_POINTS);
    glColor3f(1,0,0);
    glVertex3f(A[0],A[1],A[2]);
    glColor3f(0,1,0);
    glVertex3f(B[0],B[1],B[2]);
    glEnd();

    glLineWidth(3);
    glColor3f(0,0,1);
    glBegin(GL_LINES);
    glVertex3f(A[0],A[1],A[2]);
    glVertex3f(B[0],B[1],B[2]);
    glEnd();

    glEnable(GL_LIGHTING);
    glColor3f(0.8,0.8,0);
    glBegin(GL_TRIANGLES);
    for(set< size_t > :: const_iterator it = myedge.second.begin(); it != myedge.second.end(); it++)
        Draw_Face_Normal(*it);
    glEnd();

}
void NeighborMesh :: DrawEdge(size_t i)
{
    if(i > Edges.size() ) return;

    map <pair<size_t,size_t>,set<size_t> > :: const_iterator it(Edges.begin());
    for(size_t j=0;j<i;j++, it++){}
    DrawEdge (it);

}

void  NeighborMesh :: DrawBoundaryEdges() const
{
    //Illustrative function
    //span edges which have a set a adjacent faces with 0 or 1 element

    map <pair<size_t,size_t>,set<size_t> > :: const_iterator it;
    
    glLineWidth(2);
    glDisable(GL_LIGHTING);
    for(it = Edges.begin(); it != Edges.end(); it++)
    {
        pair < pair<size_t,size_t>,set<size_t> > myedge = *it;

        if(myedge.second.size()<2)  {
            glColor3f(1, 0, 0);
                        
        }
        else if (myedge.second.size() == 2) {
            continue;
            glColor3f(1, 1, 0);            
        } 
        else if(myedge.second.size() > 2) {
            
            // Couleur verte pour identifier les structures non-manifold / jonctions triples
            glColor3f(0.0f, 0.8f, 0.0f);

            std::set<size_t> sharedFaces = myedge.second;

            glBegin(GL_TRIANGLES);
            for (int faceIdx : sharedFaces) {
                // Récupération de la face (Vector3i contenant les indices des 3 sommets)
                const auto& f = faces[faceIdx];

                // Dessin des 3 sommets de la face
                // On utilise vertices[index] pour obtenir les coordonnées Eigen::Vector3d
                glVertex3dv(vertices[f.x()].data());
                glVertex3dv(vertices[f.y()].data());
                glVertex3dv(vertices[f.z()].data());

                //cout << f.transpose() << endl;
            }
            glEnd();
            glColor3f(0.0f, 0.8f, 0.0f);


        }

        Vector3d A(vertices[myedge.first.first]), B(vertices[myedge.first.second]);
        glBegin(GL_LINES);
        glVertex3f(A[0], A[1], A[2]);
        glVertex3f(B[0], B[1], B[2]);
        glEnd();
        glPointSize(5);
        glBegin(GL_POINTS);
        glVertex3f(A[0], A[1], A[2]);
        glVertex3f(B[0], B[1], B[2]);
        glEnd();
    }
}

/**
 * @brief Computes the geodesic distance field along mesh edges from a source vertex.
 * @details Implements a highly optimized, standard Dijkstra propagation algorithm
 * using a min-heap priority queue. This brings the time complexity down from
 * a brute-force approach to O((E + V) log V), drastically reducing computation times
 * on dense implicit CAD meshes.
 * @param A The index of the source seed vertex.
 */
void NeighborMesh::BuildDistanceLabels(size_t A)
{
    // Phase 1: Pre-allocate and initialize the global labels distance field
    Labels.assign(vertices.size(), 1e30); // Using 1e30 as an analytical infinity boundary proxy
    Labels[A] = 0.0;                      // Zero distance to self

    // Phase 2: Define and allocate hardware-efficient min-heap structures
    // Struct mapping a vertex potential distance to its structural index configuration
    typedef std::pair<double, size_t> DistancePair;

    // Core priority queue configured as a Min-Heap layout via std::greater
    std::priority_queue<DistancePair, std::vector<DistancePair>, std::greater<DistancePair>> pq;

    // Seed the priority queue with the root execution anchor
    pq.push({ 0.0, A });

    // Phase 3: Main Dijkstra relaxation and wave propagation loop
    while (!pq.empty())
    {
        // 1. Extract the unvalidated active node with the shortest temporary distance path
        const double current_dist = pq.top().first;
        const size_t current_vertex = pq.top().second;
        pq.pop();

        // 2. Early Skip Optimization: if a shorter definitive path was already found, skip processing
        if (current_dist > Labels[current_vertex])
        {
            continue;
        }

        // 3. Relax adjacent edges utilizing the pre-compiled P2P connectivity topology map
        for (size_t neighbor : P2P_Neigh[current_vertex])
        {
            // Compute explicit local Euclidean edge length segment distance
            const double edge_length = (vertices[current_vertex] - vertices[neighbor]).norm();
            const double new_path_dist = current_dist + edge_length;

            // Relaxation criterion: check if the new path sequence shortens historical boundaries
            if (Labels[neighbor] > new_path_dist)
            {
                Labels[neighbor] = new_path_dist;
                pq.push({ new_path_dist, neighbor }); // Push updated candidate onto the heap workspace
            }
        }
    }
}

// You can assign a label (a value) to the points 
// ==> allows for colored rendering of the mesh (implicit function, error, etc.)

void NeighborMesh :: SetColorsFromLabels()
{
    // brute force approach assuming that values are uniformly distributed (no outlier)
    if (Labels.empty()) return;

    colors.clear();

    // get min and max labels
    double Lmin(Labels[0]),Lmax(Labels[0]);
    for(size_t i=1; i<Labels.size();i++)
    {
        Lmin = min(Lmin, Labels[i]);
        Lmax = max(Lmax, Labels[i]);
    }

    colors.clear();
    //translate values into [0,1], then convert into color, and store
    for(size_t i=0; i<Labels.size();i++)
    {
        //adds the color to be attached to the point:

        //colors.push_back( DoubleToColor ( (Labels[i] - Lmin )/(Lmax-Lmin) ) );
    }
}

//computes and draws points neighborhoods up to ring n

void NeighborMesh :: IllustratePointNeighborhoodComputation(size_t p_index, size_t n) const
{
    glPointSize(10);
    for(size_t i=0; i<n; i++)
    {
        set <size_t> tmp = GetP2P_Neigh(p_index, i);
        if (i%2 == 0)    glColor3f(1,1-i/double(n),0);
        else glColor3f(0,1-i/double(n),1);
        DrawPoints ( tmp );
    }
}

//computes and draws faces neighborhoods up to ring n

void NeighborMesh :: IllustrateFaceNeighborhoodComputation(size_t f_index, size_t n) const
{

    for(size_t i=0; i<n; i++)
    {
        set <size_t> tmp = GetF2F_Neigh(f_index, i);
        if (i%2 == 0)    glColor3f(1,1-i/double(n),0);
        else glColor3f(0,1-i/double(n),1);
        glBegin(GL_TRIANGLES);
        DrawFaces ( tmp );
        glEnd();
    }
}

// to visualize several edges, their extremities, and attached faces

void  NeighborMesh :: IllustrateEdges( size_t n) const
{
    map< pair<size_t,size_t>, set<size_t> > :: const_iterator it (Edges.begin());
    size_t i(0);
    while (i<n)
    {
        DrawEdge(it) ;

        for (size_t j=0; j<Edges.size()/n; j++, it++){}
        //it = it + Edges.size()/n;
        i++;
    }
}

// draw point neighbors
void  NeighborMesh :: IllustrateP2P_Neigh( size_t n) const
{
    for(size_t i=0; i<n; i++)
    {
        DrawP2P_Neigh( i * int(P2P_Neigh.size()/n) );
    }

}

//draw point to face neighbors

void   NeighborMesh :: IllustrateP2F_Neigh( size_t n) const
{
    for(size_t i=0; i<n; i++)
    {
        DrawP2F_Neigh( i * int(P2F_Neigh.size()/n) );
    }
}

//draw face to face neighbors

void   NeighborMesh :: IllustrateF2F_Neigh( size_t n) const
{
    for(size_t i=0; i<n; i++)
    {
        DrawF2F_Neigh( i * int(F2F_Neigh.size()/n) );
    }

}




/**
 * @brief Classifies if a specific triangular face is obtuse and identifies its apex.
 * @details Evaluates the internal angles of the triangle using computationally efficient
 * directional dot product signs, bypassing expensive trigonometric calls or square roots.
 * @param f_index The target face index inside the mesh topology.
 * @return size_t The vertex index where the obtuse angle resides, or static_cast<size_t>(-1)
 * if the triangle is acute or right-angled.
 */
size_t NeighborMesh::IsObtuse(size_t f_index) const
{
    assert(f_index < faces.size());
    const Eigen::Vector3i F(faces[f_index]);

    for (size_t i = 0; i < 3; ++i)
    {
        const Eigen::Vector3d V1(vertices[F[(i + 1) % 3]] - vertices[F[i]]);
        const Eigen::Vector3d V2(vertices[F[(i + 2) % 3]] - vertices[F[i]]);

        // Mathematical criterion: cos(theta) < 0 <=> theta > 90 degrees
        if (V1.dot(V2) < 0.0)
        {
            return static_cast<size_t>(F[i]);
        }
    }

    return static_cast<size_t>(-1);
}


//compute 1 shortest path and draw it
//does not handle multiple/equivalent paths
void NeighborMesh :: DrawDijkstra (size_t startPointIndex, size_t endPointIndex)
{
    //Compute cumulated distances from the starting point if needed
    if (Labels.empty()) BuildDistanceLabels(startPointIndex);

    //unfold result from the end
    size_t reachedPointIndex = endPointIndex;
    glDisable(GL_LIGHTING);
    glLineWidth(5);
    glColor3f(0,1,0);
    glBegin(GL_LINE_STRIP);
    while (reachedPointIndex != startPointIndex)
    {
        //extract reachPointIndex's neighborhood
        set <size_t> N = P2P_Neigh[reachedPointIndex];

        Vector3d P (vertices[reachedPointIndex]);

        //checks for neighbors' label
        for (auto it = N.begin(); it != N.end(); it++)
        {
            Vector3d P2(vertices[*it]);
            if (Labels[*it] + (P-P2).norm() == Labels[reachedPointIndex])
            {
                //we have found the previous point
                glVertex3d (P[0],P[1],P[2]);
                glVertex3d (P2[0],P2[1],P2[2]);

                reachedPointIndex = *it; //update reached point
                it = N.end(); //exit for loop


            }
        }
    }

    glEnd();
    glEnable(GL_LIGHTING);
    glLineWidth(1);

}

void NeighborMesh::AddFaceToEdges(size_t fIdx) {
    const auto& f = faces[fIdx];
    size_t v[3] = { (size_t)f.x(), (size_t)f.y(), (size_t)f.z() };
    for (int i = 0; i < 3; ++i) {
        size_t v1 = v[i], v2 = v[(i + 1) % 3];
        if (v1 > v2) std::swap(v1, v2);
        Edges[{v1, v2}].insert(fIdx);
    }
}

void NeighborMesh::RemoveFaceFromEdges(size_t fIdx) {
    const auto& f = faces[fIdx];
    size_t v[3] = { (size_t)f.x(), (size_t)f.y(), (size_t)f.z() };
    for (int i = 0; i < 3; ++i) {
        size_t v1 = v[i], v2 = v[(i + 1) % 3];
        if (v1 > v2) std::swap(v1, v2);

        auto it = Edges.find({ v1, v2 });
        if (it != Edges.end()) {
            it->second.erase(fIdx);
            // Si l'arête n'est plus utilisée par aucune face, on la supprime
            if (it->second.empty()) {
                Edges.erase(it);
            }
        }
    }
}


size_t NeighborMesh::GetOppositeVertex(size_t fIdx, size_t v1, size_t v2) const {
    // On récupère la face (Vector3i contenant les 3 index de sommets)
    const Eigen::Vector3i& f = faces[fIdx];

    // On vérifie quel index n'est pas l'un des deux sommets de l'arête
    if ((size_t)f.x() != v1 && (size_t)f.x() != v2) return (size_t)f.x();
    if ((size_t)f.y() != v1 && (size_t)f.y() != v2) return (size_t)f.y();
    if ((size_t)f.z() != v1 && (size_t)f.z() != v2) return (size_t)f.z();

    // Cas de sécurité (ne devrait jamais arriver sur un maillage sain)
    return (size_t)-1;
}




// Fonction utilitaire pour l'intersection Segment/Triangle
bool IntersectEdgeTriangle(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
    const Eigen::Vector3d& v0, const Eigen::Vector3d& v1, const Eigen::Vector3d& v2,
    Eigen::Vector3d& outP) {
    Eigen::Vector3d e1 = v1 - v0;
    Eigen::Vector3d e2 = v2 - v0;
    Eigen::Vector3d dir = p2 - p1;
    Eigen::Vector3d h = dir.cross(e2);
    double a = e1.dot(h);

    if (std::abs(a) < 1e-10) return false;

    double f = 1.0 / a;
    Eigen::Vector3d s = p1 - v0;
    double u = f * s.dot(h);
    if (u < 0.0 || u > 1.0) return false;

    Eigen::Vector3d q = s.cross(e1);
    double v = f * dir.dot(q);
    if (v < 0.0 || u + v > 1.0) return false;

    double t = f * e2.dot(q);
    if (t > 0.0 && t < 1.0) { // Intersection sur le segment
        outP = p1 + dir * t;
        return true;
    }
    return false;
}




// Fuse similar or close vertices generated by MC
void NeighborMesh::WeldVertices(double epsilon_weld) {
    std::vector<Eigen::Vector3d> uniqueVertices;
    std::map<std::tuple<long long, long long, long long>, int> posToIdx;
    std::vector<int> oldToNew(vertices.size());

    // 1. Identify identical points
    for (size_t i = 0; i < vertices.size(); ++i) {
        Eigen::Vector3d p = vertices[i];
        // Discretization for spatial hashing key
        auto key = std::make_tuple(
            (long long)(p.x() / epsilon_weld),
            (long long)(p.y() / epsilon_weld),
            (long long)(p.z() / epsilon_weld)
        );

        if (posToIdx.find(key) == posToIdx.end()) {
            posToIdx[key] = (int)uniqueVertices.size();
            uniqueVertices.push_back(p);
        }
        oldToNew[i] = posToIdx[key];
    }

    // 2. Update faces with new indices
    std::vector<Vector3i> newFaces;
    for (const auto& f : faces) {
        Vector3i newF(oldToNew[f.x()], oldToNew[f.y()], oldToNew[f.z()]);
        // Remove degenerate triangles (zero-area) created by vertex welding
        if (newF.x() != newF.y() && newF.y() != newF.z() && newF.z() != newF.x()) {
            newFaces.push_back(newF);
        }
    }

    // 3. Replace data and reconstruct topology
    this->vertices = uniqueVertices;
    this->faces = newFaces;

    // Force complete reconstruction of neighborhood structures
    this->Build_P2P_Neigh();
    this->Build_P2F_Neigh();
    this->Build_Edges();

    std::cout << "\t[Weld] Vertices reduced: " << oldToNew.size() << " -> " << vertices.size() << std::endl;
}


void NeighborMesh::ReplaceVertexIndex(int oldIdx, int newIdx) {
    if (oldIdx == newIdx) return;

    // On parcourt toutes les faces qui utilisent l'ancien index
    // P2F_Neigh est parfait pour ça si tu l'as déjà build
    for (int fIdx : P2F_Neigh[oldIdx]) {
        Vector3i& f = faces[fIdx];
        for (int i = 0; i < 3; ++i) {
            if (f[i] == oldIdx) f[i] = newIdx;
        }
    }
}



void NeighborMesh::ClearAll()
{
    // 1. Appel du Clear de la classe parente (Mesh)
    // Supposons que Mesh::ClearAll() vide déjà vertices, faces, normals
    Mesh::ClearAll();

    // 2. Nettoyage des vecteurs de voisinage (P2P, P2F, F2F)
    // On utilise clear() puis shrink_to_fit() pour libérer réellement la mémoire vive
    P2P_Neigh.clear();
    P2F_Neigh.clear();
    F2F_Neigh.clear();

    // 3. Nettoyage de la structure d'arêtes (la map est très gourmande)
    Edges.clear();

    // 4. Nettoyage des étiquettes et données de calcul
    Labels.clear();
    

    // 6. Reset des flags d'adjacence hérités de Mesh
    // (A adapter selon les noms exacts dans ta classe Mesh)
    //this->adjacence_ok = false;
}

void NeighborMesh::RemoveUnusedVertices() {
    if (vertices.empty()) return;

    // 1. Mark referenced vertices
    std::vector<bool> isUsed(vertices.size(), false);
    for (const auto& f : faces) {
        isUsed[(size_t)f.x()] = true;
        isUsed[(size_t)f.y()] = true;
        isUsed[(size_t)f.z()] = true;
    }

    // 2. Build map for index remapping (Old Index -> New Index)
    std::vector<size_t> newIndices(vertices.size(), 0);
    std::vector<Eigen::Vector3d> newVertices;
    newVertices.reserve(vertices.size());

    for (size_t i = 0; i < vertices.size(); ++i) {
        if (isUsed[i]) {
            newIndices[i] = newVertices.size();
            newVertices.push_back(vertices[i]);
        }
    }

    // 3. Update face indices using the remapping table
    for (auto& f : faces) {
        f.x() = (int)newIndices[(size_t)f.x()];
        f.y() = (int)newIndices[(size_t)f.y()];
        f.z() = (int)newIndices[(size_t)f.z()];
    }

    // 4. Swap old vertices with the cleaned array using move semantics
    size_t removedCount = vertices.size() - newVertices.size();
    vertices = std::move(newVertices);

    if (removedCount > 0) {
        std::cout << "[CLEANUP] Removed " << removedCount << " unused vertices." << std::endl;
    }
}