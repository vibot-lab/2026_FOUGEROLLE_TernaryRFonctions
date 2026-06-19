/***************************************************************************
                                    Mesh.h
                             -------------------
    update              : 2002-10-11
    copyright           : (C) 2002 by Michaël ROY
    email               : michaelroy@users.sourceforge.net

    update(s)			: Yohan Fougerolle - 2005 until now
	email				: yohan.fougerolle@u-bourgogne.fr
    
	Warning				: handle only triangular faces

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef _MESH_
#define _MESH_

 // 1. Forward Declarations required for MC mesh constructions
namespace RCore {
	class SphericalEvaluator; 
}
class ImplicitCylinder; 

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <set>
#include <map>
#include <algorithm>
#include <utility>
#include <cassert>
#include <cstring>

#include "ImplicitObjects.h"
#include "Rfunctions.h"

#define ZERO 1e-9
#define EPSILON 1e-4
#define ITERATION_MAX 10
#define NEGATIVE -1
#define POSITIVE 1
#define FLAT_TRIANGLE_AREA 0.0001

//Display Modes
#define NO_DISPLAY_PARAM			0000010
#define NORMAL_DISPLAY_MODE			0000020
#define FACE_NORMAL_DISPLAY_MODE	0000021
#define RGB_DISPLAY_MODE			0000030
#define FULL_DISPLAY_MODE			0000040
#define ONLY_VERTEX					1000000
#define FACE_NORMAL					1000100
#define FACE_NORMAL_RGB				1000200
#define VERTEX_NORMAL				1000300
#define VERTEX_NORMAL_RGB			1000400



//--
//
// AttributeBinding
//
//--
enum AttributeBinding
{
	UNKNOWN_BINDING,
	PER_VERTEX_BINDING,
	PER_FACE_BINDING
};

//--
//
// FileFormat
//
//--
enum FileFormat
{
	UNKNOWN_FILE,
	IV_FILE,
	SMF_FILE,
	STL_ASCII_FILE,
	STL_BINARY_FILE,
	VRML_1_FILE,
	RANGE_FILE,
	OFF_FILE_1,
	OFF_FILE_2,
	CLOUD_FILE
};

//--
//
// Utility function
//
//--

// Upper case string of the string s
extern std::string UpperCase( const std::string& s );

//--
//
// Mesh
//
//--
class Mesh
{

	//--
	//
	// Member Data
	//
	//--

	// everything is public : it's bad, but convenient for our project
	// feel free to change from public to private/protected if you want

	public:

		RCore::SphericalEvaluator* TernarySphericalOperator;
		
		std::vector<VertexResearchData> vertex_results;
       
        //display vertex and face numbers in the OpenGL window
		void Print_Stats();

		// Vertex array
		std::vector<Eigen::Vector3d> vertices;

		//associated index function to each point
		std::vector <int > vertex_function_index;

		// Face array
		std::vector<Eigen::Vector3i> faces;

		// Color array
		std::vector<Eigen::Vector3d> colors;

		// Texture coordinate array ... probably useless for you in this project
		std::vector<Eigen::Vector2d> textures;

		// Face normal array
		std::vector<Eigen::Vector3d> face_normals;

		// Vertex normal array
		std::vector<Eigen::Vector3d> vertex_normals;

		// Texture file name ... probably useless for you in this project
		std::string texture_name;

		// Color binding (per vertex or per face)
		AttributeBinding color_binding;


    //--
	//
	// Member functions
	//
	//--

		void Draw(int Drawing_Mode);
        void Draw_Default(int i);
        void Draw_Face_Normal(int i) const;
		void Draw_Face_Normal_Rgb(int i);
		void Draw_Vertex_Normal(int);
		void Draw_Vertex_Normal_Rgb(int);


		//--
		//
		// Constructor / Destructor
		//
		//==
		Mesh();

		virtual~Mesh();

		//==
		//
		// File input/output
		//
		//==

		// Read mesh from a file
		bool ReadFile( const std::string& file_name );

		// Write mesh in a file
		bool WriteFile( const std::string& file_name, const FileFormat& file_format=VRML_1_FILE ) const;

		//==
		//
		// Vertex Interface
		//
		//==

		// Vertex number
		inline int VertexNumber() const {
			return (int)vertices.size();
		}

		// Vertex array
		inline std::vector<Eigen::Vector3d>& Vertices() {
			return vertices;
		}

		// Vertex array (constant)
		inline const std::vector<Eigen::Vector3d>& Vertices() const {
			return vertices;
		}

		// Add vertex v in vertex array
		inline void AddVertex( const Eigen::Vector3d& v ) {
			vertices.push_back( v );
		}

		// Clear vertex array
		inline void ClearVertices() {
			vertices.clear();
		}

		// Vertex #i
		inline Eigen::Vector3d& Vertex(int i) {
			assert( (i>=0) && (i<VertexNumber()) );
			return vertices[i];
		}

		// Vertex #i (constant)
		inline const Eigen::Vector3d& Vertex(int i) const {
			assert( (i>=0) && (i<VertexNumber()) );
			return vertices[i];
		}

		// Vertex of face #f with index #v
		// v is in range 0 to 2
		inline Eigen::Vector3d& Vertex(int f, int v) {
			assert( (f>=0) && (f<FaceNumber()) );
			assert( (v>=0) && (v<=2) );
			assert( (faces[f][v]>=0) && (faces[f][v]<VertexNumber()) );
			return vertices[faces[f][v]];
		}

		// Vertex of face #f with index #v (constant)
		// v is in range 0 to 2
		inline const Eigen::Vector3d& Vertex(int f, int v) const {
			assert( (f>=0) && (f<FaceNumber()) );
			assert( (v>=0) && (v<=2) );
			assert( (faces[f][v]>=0) && (faces[f][v]<VertexNumber()) );
			return vertices[faces[f][v]];
		}

		//==
		//
		// Face Interface
		//
		//==

		// Face number
		inline int FaceNumber() const {
			return (int)faces.size();
		}

		// Face array
		inline std::vector<Eigen::Vector3i>& Faces() {
			return faces;
		}

		// Face array (constant)
		inline const std::vector < Eigen::Vector3i > & Faces() const {
			return faces;
		}

		// Add face f to face array
		inline void AddFace( const Eigen::Vector3i& f ) {
			faces.push_back( f );
		}

		// Clear face array
		inline void ClearFaces() {
			faces.clear();
		}

		// Face #i
		inline Eigen::Vector3i& Face(int i) {
			assert( (i>=0) && (i<FaceNumber()) );
			return faces[i];
		}

		// Face #i (constant)
		inline const Eigen::Vector3i& Face(int i) const {
			assert( (i>=0) && (i<FaceNumber()) );
			return faces[i];
		}

		// Index #v of face #f
		// v is in range 0 to 2
		inline int& FaceIndex(int f, int v) {
			assert( (f>=0) && (f<FaceNumber()) );
			assert( (v>=0) && (v<=2) );
			return faces[f][v];
		}

		// Index #v of face #f (constant)
		// Index #v is in range 0 to 2
		inline const int& FaceIndex(int f, int v) const {
			assert( (f>=0) && (f<FaceNumber()) );
			assert( (v>=0) && (v<=2) );
			return faces[f][v];
		}

		// Test if face #f has vertex #v
		inline bool FaceHasVertex( int f, int v ) const {
			return (FaceIndex(f,0) == v) || (FaceIndex(f,1) == v) || (FaceIndex(f,2) == v);
		}

		//==
		//
		// Color Interface
		//
		//==

		// Color binding
		// Valid binding are "per vertex" or "per face"
		inline AttributeBinding& ColorBinding() {
			return color_binding;
		}

		// Color binding (constant)
		// Valid binding are "per vertex" or "per face"
		inline const AttributeBinding& ColorBinding() const {
			return color_binding;
		}

		// Color number
		inline int ColorNumber() const {
			return (int)colors.size();
		}

		// Color array
		inline std::vector<Eigen::Vector3d>& Colors() {
			return colors;
		}

		// Color array (constant)
		inline const std::vector<Eigen::Vector3d>& Colors() const {
			return colors;
		}

		// Add color c in color array
		inline void AddColor( const Eigen::Vector3d& c ) {
			colors.push_back( c );
		}

		// Clear color array
		inline void ClearColors() {
			colors.clear();
		}

		// Color #i
		inline Eigen::Vector3d& Color(int i) {
			assert( (i>=0) && (i<ColorNumber()) );
			return colors[i];
		}

		// Color #i (constant)
		inline const Eigen::Vector3d& Color(int i) const {
			assert( (i>=0) && (i<ColorNumber()) );
			return colors[i];
		}

		// Color of face #f with index #v
		// v is in range 0 to 2
		inline Eigen::Vector3d& Color(int f, int v) {
			assert( (f>=0) && (f<FaceNumber()) );
			assert( (v>=0) && (v<=2) );
			assert( (faces[f][v]>=0) && (faces[f][v]<ColorNumber()) );
			return colors[faces[f][v]];
		}

		// Color of face #f with index #v (constant)
		// v is in range 0 to 2
		inline const Eigen::Vector3d& Color(int f, int v) const {
			assert( (f>=0) && (f<FaceNumber()) );
			assert( (v>=0) && (v<=2) );
			assert( (faces[f][v]>=0) && (faces[f][v]<ColorNumber()) );
			return colors[faces[f][v]];
		}

		//==
		//
		// Texture Interface ... probably useless for this project
		//
		//==

		// Texture file name
		inline std::string& TextureName() {
			return texture_name;
		}

		// Texture file name (constant)
		inline const std::string& TextureName() const {
			return texture_name;
		}

		// Clear texture file name
		inline void ClearTextureName() {
			texture_name = "";
		}

		// Texture coordinate number
		inline int TextureNumber() const {
			return (int)textures.size();
		}

		// Texture coordinate array
		inline std::vector<Eigen::Vector2d>& Textures() {
			return textures;
		}

		// Texture coordinate array (constant)
		inline const std::vector<Eigen::Vector2d>& Textures() const {
			return textures;
		}

		// Add texture coordinate t in texture coordinate array
		inline void AddTexture( const Eigen::Vector2d& t ) {
			textures.push_back( t );
		}

		// Clear texture coordinate array
		inline void ClearTextures() {
			textures.clear();
		}

		// Texture coordinate #i
		inline Eigen::Vector2d& Texture(int i) {
			assert( (i>=0) && (i<TextureNumber()) );
			return textures[i];
		}

		// Texture coordinate #i (constant)
		inline const Eigen::Vector2d& Texture(int i) const {
			assert( (i>=0) && (i<TextureNumber()) );
			return textures[i];
		}

		// Texture coordinate of face #f with index #v
		// v is in range 0 to 2
		inline Eigen::Vector2d& Texture(int f, int v) {
			assert( (f>=0) && (f<FaceNumber()) );
			assert( (v>=0) && (v<=2) );
			assert( (faces[f][v]>=0) && (faces[f][v]<TextureNumber()) );
			return textures[faces[f][v]];
		}

		// Texture coordinate of face #f with index #v (constant)
		// v is in range 0 to 2
		inline const Eigen::Vector2d& Texture(int f, int v) const {
			assert( (f>=0) && (f<FaceNumber()) );
			assert( (v>=0) && (v<=2) );
			assert( (faces[f][v]>=0) && (faces[f][v]<TextureNumber()) );
			return textures[faces[f][v]];
		}

		//==
		//
		// Face Normal Interface
		//
		//==

		// Face normal number
		inline int FaceNormalNumber() const {
			return (int)face_normals.size();
		}

		// Face normal array
		inline std::vector<Eigen::Vector3d>& FaceNormals() {
			return face_normals;
		}

		// Face normal array (constant)
		inline const std::vector<Eigen::Vector3d>& FaceNormals() const {
			return face_normals;
		}

		// Add face normal n to face array
		inline void AddFaceNormal( const Eigen::Vector3d& n ) {
			face_normals.push_back( n );
		}

		// Clear face array
		inline void ClearFaceNormals() {
			face_normals.clear();
		}

		// Face normal #i
		inline Eigen::Vector3d& FaceNormal(int i) {
			assert( (i>=0) && (i<FaceNormalNumber()) );
			return face_normals[i];
		}

		// Face normal #i (constant)
		inline const Eigen::Vector3d& FaceNormal(int i) const {
			assert( (i>=0) && (i<(int)FaceNormalNumber()) );
			return face_normals[i];
		}

		//--
		//
		// Vertex Normal Interface
		//
		//--

		// Vertex normal number
		inline int VertexNormalNumber() const {
			return (int)vertex_normals.size();
		}

		// Vertex normal array
		inline std::vector<Eigen::Vector3d>& VertexNormals() {
			return vertex_normals;
		}

		// Vertex normal array (constant)
		inline const std::vector<Eigen::Vector3d>& VertexNormals() const {
			return vertex_normals;
		}

		// Add vertex normal n to vertex normal array
		inline void AddVertexNormal( const Eigen::Vector3d& n ) {
			vertex_normals.push_back( n );
		}

		// Clear vertex normal array
		inline void ClearVertexNormals() {
			vertex_normals.clear();
		}

		// Vertex normal #i
		inline Eigen::Vector3d& VertexNormal(int i) {
			assert( (i>=0) && (i<VertexNormalNumber()) );
			return vertex_normals[i];
		}

		// Vertex normal #i (constant)
		inline const Eigen::Vector3d& VertexNormal(int i) const {
			assert( (i>=0) && (i<VertexNormalNumber()) );
			return vertex_normals[i];
		}

		// Vertex normal of face #f with index #v
		// v is in range 0 to 2
		inline Eigen::Vector3d& VertexNormal(int f, int v) {
			assert( (f>=0) && (f<FaceNumber()) );
			assert( (v>=0) && (v<=2) );
			assert( (faces[f][v]>=0) && (faces[f][v]<VertexNormalNumber()) );
			return vertex_normals[faces[f][v]];
		}

		// Vertex normal of face #f with index #v (constant)
		// v is in range 0 to 2
		inline const Eigen::Vector3d& VertexNormal(int f, int v) const {
			assert( (f>=0) && (f<FaceNumber()) );
			assert( (v>=0) && (v<=2) );
			assert( (faces[f][v]>=0) && (faces[f][v]<VertexNormalNumber()) );
			return vertex_normals[faces[f][v]];
		}

		//--
		//
		// Normal computation
		//
		//--

		// Compute face normals
		void ComputeFaceNormals();

		// Compute vertex normals
		// Assume that face normals are computed
		void ComputeVertexNormals();

		// Compute raw normal of face #f
		inline Eigen::Vector3d ComputeRawFaceNormal( int f ) const {
			return (Vertex(f,1)-Vertex(f,0)) .cross ( (Vertex(f,2)-Vertex(f,0)) );
		}

		// Compute raw normal of face {va, vb, vc}
		inline Eigen::Vector3d ComputeRawFaceNormal( int va, int vb, int vc ) const {
			return (Vertex(vb)-Vertex(va)) .cross ( (Vertex(vc)-Vertex(va)) );
		}

		// Compute unit normal of face #f
		inline Eigen::Vector3d ComputeFaceNormal( int f ) const {
			assert( (f>=0) && (f<(int)FaceNumber()) );
			return ComputeRawFaceNormal(f).normalized();
		}

		// Compute unit normal of face {va, vb, vc}
		inline Eigen::Vector3d ComputeFaceNormal( int va, int vb, int vc ) const {
			return ComputeRawFaceNormal(va, vb, vc).normalized();
		}

		// Compute area of face #i
		inline double ComputeFaceArea( int i ) const {
			return 0.5 * ComputeRawFaceNormal(i).norm();
		}

		// Compute area of face {va, vb ,vc}
		inline double ComputeFaceArea( int va, int vb, int vc ) const {
			return 0.5 * ComputeRawFaceNormal(va, vb, vc).norm();
		}

		//clean
		void GetGeometricStatistics() const ;
		void ClearAll();

};

#endif // _MESH_

