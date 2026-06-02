/***************************************************************************
                                  Mesh.cpp
                             -------------------
    update               : 2002-12-05
    copyright            : (C) 2002 by Michaël ROY
    email                : michaelroy@users.sourceforge.net
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include <fstream>
#include <iostream>
#include <GL/glut.h>
#include <map>
#include "Mesh.h"
#include "ImplicitObjects.h"
using namespace std;
using namespace Eigen;



//--
//
// StringToBinding
//
//--

// Convert a string to an attribute binding
static AttributeBinding StringToBinding(const string& s)
{
	if (s == "PER_VERTEX_INDEXED")
	{
		return PER_VERTEX_BINDING;
	}
	if (s == "PER_FACE_INDEXED")
	{
		return PER_FACE_BINDING;
	}
	return UNKNOWN_BINDING;
}

//--
//
// BindingToString
//
//--
// Convert an attribute binding to a string
static string BindingToString(const AttributeBinding& ab)
{
	if (ab == PER_VERTEX_BINDING)
	{
		return "PER_VERTEX_INDEXED";
	}
	if (ab == PER_FACE_BINDING)
	{
		return "PER_FACE_INDEXED";
	}
	return "UNKNOWN_BINDING";
}



//--
//
// ReadIv : for reading meshes in iv, vrml format
//
//--

// Read OpenInventor/VRML1.0 file
bool ReadIv(Mesh& mesh, const string& file_name) {

	cout << "Reading IV/VRML" << endl;
	ifstream file; // File to read
	string line, new_line; // Lines to process
	string word, previous_word; // Words to process
	string::iterator it; // Iterator for lines
	size_t pos; // String position
	int nlbrack(0), nrbrack(0); // Left/Right delimiter counter
	int level(0); // Node level
	int ixyz(0); // Coordinate index
	Vector3d vec3d; // Temp vector 3D (vertex, color, normal)
	Vector3i vec3i; // Temp vector 3I (face)
	Vector2d vec2d; // Temp vector 2D (texture coordinate)
	AttributeBinding normal_binding(PER_VERTEX_BINDING); // Temp normal binding
	AttributeBinding texture_binding(PER_VERTEX_BINDING); // Temp texture binding
	vector<string> node(10); // nodes - 10 levels by default

	// Open file
	file.open(file_name.c_str());

	// Test if file is open
	if (file.is_open() == false)
	{
		cout << "No file found" << endl;
		return false;
	}

	//--
	// Read file header
	//--

	// Get first line of the file
	getline(file, line);

	// Try to find #Inventor in the line
	pos = line.find("#Inventor");
	if (pos > line.length())
	{
		// Else try to find #VRML V1.0 in the line
		pos = line.find("#VRML V1.0");
		if (pos > line.length())
		{
			cout << "pas VRML" << endl;
			return false;
		}
	}

	// Verify file is ascii format
	pos = line.find("ascii");
	if (pos > line.length())
	{
		return false;
	}

	// Initialize the mesh
	mesh.ClearAll();

	//--
	// Read the file until the end reading line by line
	//--
	while (getline(file, line))
	{
		// Replace all comas by space
		replace(line.begin(), line.end(), ',', ' ');

		// Initialize new_line
		new_line.erase(new_line.begin(), new_line.end());

		//--
		// Buffer brackets and braces by space and copy in new_line
		//--
		it = line.begin();
		while (it != line.end())
		{

			if (((*it) == '{') || ((*it) == '}') || ((*it) == '[') || ((*it) == ']') || ((*it) == '#'))
			{
				new_line += ' ';
				new_line += *it++;
				new_line += ' ';
			}
			else
			{
				new_line += *it++;
			}
		}

		//--
		// Analyse each word in the line
		//--
		istringstream word_stream(new_line);
		while (word_stream >> word)
		{
			//--
			// Left bracket or brace
			//--
			if ((word == "{") || (word == "["))
			{
				// Increment left deliminter number
				nlbrack++;
				// Get level number
				level = nlbrack - nrbrack;
				// Save level name
				if (level > (int)node.size())
				{
					node.push_back(UpperCase(previous_word));
				}
				else
				{
					node[level] = UpperCase(previous_word);
				}
				// Initialize coordinate index
				ixyz = 0;
			}

			//--
			// Right bracket or brace
			//--
			else if ((word == "}") || (word == "]"))
			{
				// Increment right deliminter number
				nrbrack++;
				// Get level number
				level = nlbrack - nrbrack;
				// Sanity test
				if (level < 0)
				{
					mesh.ClearAll();
					return false;
				}
			}

			//--
			// Comment
			//--
			else if (word == "#")
			{
				// Save current word
				previous_word = word;
				// Next line
				break;
			}

			//--
			// COORDINATE3
			//--
			else if (node[level] == "COORDINATE3")
			{
			}

			//--
			// INDEXEDFACESET
			//--
			else if (node[level] == "INDEXEDFACESET")
			{
			}

			//--
			// MATERIAL
			//--
			else if (node[level] == "MATERIAL")
			{
			}

			//--
			// MATERIALBINDING
			//--
			else if (node[level] == "MATERIALBINDING")
			{
				if (previous_word == "value")
				{
					mesh.ColorBinding() = StringToBinding(word);
				}
			}

			//--
			// NORMALBINDING
			//--
			else if (node[level] == "NORMALBINDING")
			{
				if (previous_word == "value")
				{
					normal_binding = StringToBinding(word);
				}
			}

			//--
			// TEXTURECOORDINATEBINDING
			//--
			else if (node[level] == "TEXTURECOORDINATEBINDING")
			{
				if (previous_word == "value")
				{
					texture_binding = StringToBinding(word);
				}
			}

			//--
			// POINT
			//--
			else if (node[level] == "POINT")
			{
				if (node[level - 1] == "COORDINATE3")
				{
					// Get current value
					vec3d[ixyz] = atof(word.c_str());
					// Complete coordinate ?
					if (ixyz == 2)
					{
						mesh.AddVertex(vec3d);
						ixyz = 0;
					}
					else
					{
						ixyz++;
					}
				}
				else if (node[level - 1] == "TEXTURECOORDINATE2")
				{
					// Get current value
					vec2d[ixyz] = atof(word.c_str());
					// Complete coordinate ?
					if (ixyz == 1)
					{
						mesh.AddTexture(vec2d);
						ixyz = 0;
					}
					else
					{
						ixyz++;
					}
				}
			}

			//--
			// DIFFUSECOLOR
			//--
			else if (node[level] == "DIFFUSECOLOR")
			{
				if (node[level - 1] == "MATERIAL")
				{
					// Get current value
					vec3d[ixyz] = atof(word.c_str());
					// Complete coordinate ?
					if (ixyz == 2)
					{
						mesh.AddColor(vec3d);
						ixyz = 0;
					}
					else
					{
						// Next coordinate
						ixyz++;
					}
				}
			}

			//--
			// VECTOR
			//--
			else if (node[level] == "VECTOR")
			{
				if (node[level - 1] == "NORMAL")
				{
					// Get current value
					vec3d[ixyz] = atof(word.c_str());
					// Complete coordinate ?
					if (ixyz == 2)
					{
						mesh.AddVertexNormal(vec3d);
						ixyz = 0;
					}
					else
					{
						// Next coordinate
						ixyz++;
					}
				}
			}

			//--
			// TEXTURE2
			//--
			else if (node[level] == "TEXTURE2")
			{
				if (previous_word == "filename")
				{
					// Get texture filename
					mesh.TextureName() = word;
				}
			}

			//--
			// COORDINDEX
			//--
			else if (node[level] == "COORDINDEX")
			{
				if (node[level - 1] == "INDEXEDFACESET")
				{
					// -1 value
					if (ixyz == 3)
					{
						// Next face
						ixyz = 0;
						continue;
					}
					// Get value
					vec3i[ixyz] = atoi(word.c_str());
					// Complete coordinate ?
					if (ixyz == 2)
					{
						mesh.AddFace(vec3i);
					}
					// Next coordinate
					ixyz++;
				}
			}

			//--
			// MATERIALINDEX
			//--
			else if (node[level] == "MATERIALINDEX")
			{
				if (node[level - 1] == "INDEXEDFACESET")
				{
				}
			}

			//--
			// TEXTURECOORDINDEX
			//--
			else if (node[level] == "TEXTURECOORDINDEX")
			{
				if (node[level - 1] == "INDEXEDFACESET")
				{
				}
			}

			//--
			// NORMALINDEX
			//--
			else if (node[level] == "NORMALINDEX")
			{
				if (node[level - 1] == "INDEXEDFACESET")
				{
				}
			}

			// Save current word
			previous_word = word;
		}
	}

	// Check vertex normals
	if ((normal_binding != PER_VERTEX_BINDING) || (mesh.VertexNormalNumber() != mesh.VertexNumber()))
	{
		mesh.ClearVertexNormals();
	}

	// Normalize normals
	if (mesh.VertexNormalNumber() != 0)
	{
		for (int i = 0; i < mesh.VertexNormalNumber(); i++)
		{
			mesh.VertexNormal(i).normalized();
		}
	}

	// Check texture coordinates
	if ((texture_binding != PER_VERTEX_BINDING) || (mesh.TextureNumber() != mesh.VertexNumber()))
	{
		mesh.ClearTextures();
		mesh.ClearTextureName();
	}

	// Check colors
	if ((mesh.ColorBinding() == PER_FACE_BINDING) && (mesh.ColorNumber() != mesh.FaceNumber()))
	{
		mesh.ClearColors();
		mesh.ColorBinding() = UNKNOWN_BINDING;
	}
	else if ((mesh.ColorBinding() == PER_VERTEX_BINDING) && (mesh.ColorNumber() != mesh.VertexNumber()))
	{
		mesh.ClearColors();
		mesh.ColorBinding() = UNKNOWN_BINDING;
	}

	file.close();

	return true;
}

//--
//
// WriteIv
//
//--
// Write model to an OpenInventor / VRML 1.0 file

bool WriteIv(const Mesh& mesh, const string& file_name, const bool& vrml1)
{
	int i;

	// Test if mesh is valid
	if ((mesh.VertexNumber() == 0) || (mesh.FaceNumber() == 0))
	{
		cout << "No face or no vertex" << endl;
		return false;
	}

	// Open file for writing
	ofstream file(file_name.c_str());

	// Test if the file is open
	if (file.is_open() == false)
	{
		cout << "crashed opening file" << endl;
		return false;
	}

	//--
	// Write file Header
	//--

	// VRML 1.0 file header
	if (vrml1) file << "#VRML V1.0 ascii\n" << endl;
	// or OpenInventor file header
	else file << "#Inventor V2.1 ascii\n" << endl;

	// Write vertex number (comment)
	file << "# Vertices: " << mesh.VertexNumber() << endl;
	// Write face number (comment)
	file << "# Faces:    " << mesh.FaceNumber() << endl;
	file << endl;

	// Begin description
	file << "Separator {" << endl;

	//--
	// Write  vertex coordinates
	//--
	file << "    Coordinate3 {" << endl;
	file << "        point [" << endl;
	for (i = 0; i < (int)mesh.VertexNumber() - 1; i++)
	{
		file << "            " << mesh.Vertex(i) << "," << endl;
	}
	file << "            " << mesh.Vertex(mesh.VertexNumber() - 1) << endl;
	file << "        ]" << endl;
	file << "    }" << endl;

	//--
	// Write colors
	//--
	if ((mesh.ColorNumber()) && (mesh.ColorBinding() != UNKNOWN_BINDING))
	{
		// Binding
		file << "    MaterialBinding {" << endl;
		file << "        value " << BindingToString(mesh.ColorBinding()) << endl;
		file << "    }" << endl;
		// Color
		file << "    Material {" << endl;
		file << "        diffuseColor [" << endl;
		for (i = 0; i < (int)mesh.ColorNumber() - 1; i++)
		{
			file << "            " << mesh.Color(i) << "," << endl;
		}
		file << "            " << mesh.Color(mesh.ColorNumber() - 1) << endl;
		file << "        ]" << endl;
		file << "    }" << endl;
	}

	//--
	// Write vertex normals
	//--
	if (mesh.VertexNormalNumber() == mesh.VertexNumber())
	{
		// Binding
		file << "    NormalBinding {" << endl;
		file << "        value PER_VERTEX_INDEXED" << endl;
		file << "    }" << endl;
		// Vertex normals
		file << "    Normal {" << endl;
		file << "        vector [" << endl;
		for (i = 0; i < (int)mesh.VertexNormalNumber() - 1; i++)
		{
			file << "            " << mesh.VertexNormal(i) << "," << endl;
		}
		file << "            " << mesh.VertexNormal(mesh.VertexNormalNumber() - 1) << endl;
		file << "        ]" << endl;
		file << "    }" << endl;
	}

	//--
	// Write texture
	//--
	if ((mesh.TextureNumber() == mesh.VertexNumber()) && (mesh.TextureName() != ""))
	{
		// Texture file name
		file << "    Texture2 {" << endl;
		file << "        filename \"" << mesh.TextureName() << "\"" << endl;
		file << "    }" << endl;
		// Texture coordinate binding
		file << "    TextureCoordinateBinding {" << endl;
		file << "        value PER_VERTEX_INDEXED" << endl;
		file << "    }" << endl;
		// Texture coordinates
		file << "    TextureCoordinate2 {" << endl;
		file << "        point [" << endl;
		for (i = 0; i < (int)mesh.TextureNumber() - 1; i++)
		{
			file << "            " << mesh.Texture(i) << "," << endl;
		}
		file << "            " << mesh.Texture(mesh.TextureNumber() - 1) << endl;
		file << "        ]" << endl;
		file << "    }" << endl;
	}

	//--
	// Write face indices
	//--

	// Begin face index block
	file << "    IndexedFaceSet  {" << endl;

	// Index (same for vertices, colors, texture cooridnates, normals)
	file << "        coordIndex [" << endl;
	for (i = 0; i < (int)mesh.FaceNumber() - 1; i++)
	{
		file << "            " << mesh.FaceIndex(i, 0) << ", " << mesh.FaceIndex(i, 1) << ", " << mesh.FaceIndex(i, 2) << ", -1," << endl;
	}
	file << "            " << mesh.FaceIndex(i, 0) << ", " << mesh.FaceIndex(i, 1) << ", " << mesh.FaceIndex(i, 2) << ", -1" << endl;
	file << "        ]" << endl;

	// End face index block
	file << "     }" << endl;

	// End description
	file << "}" << endl;

	// Close file
	file.close();

	return true;
}





unsigned int Mesh::getFurthestPoint(unsigned int i0)
{
    map<double, unsigned int> M;
    Vector3d P0(vertices[i0]);
    double d;
    for (unsigned int i = 0; i<vertices.size(); ++i)
    {
        if ( i == i0) continue;

        d = (P0-vertices[i]).squaredNorm();
        M.insert({d, i});
    }

    return M.rbegin()->second;

}
unsigned int Mesh::getFurthestPoint(unsigned int i0, unsigned int i1)
{
    map<double, unsigned int> M;
    assert(i0 != i1);
    Vector3d P0(vertices[i0]), P1(vertices[i1]);
    double f, d0, d1;
    for (unsigned int i = 0; i<vertices.size(); ++i)
    {
        if ( i == i0 || i == i1) continue;

        d0 = (P0-vertices[i]).squaredNorm();
        d1 = (P1-vertices[i]).squaredNorm();

        f = d0 + d1 - sqrt(d0*d0+d1*d1);

        M.insert({f, i});
    }
    return M.rbegin()->second;
}

unsigned int Mesh::getFurthestPoint(unsigned int i0, unsigned int i1, unsigned int i2)
{
    assert(i0 != i1 && i0 != i2 && i1 != i2);

    Vector3d P0 = vertices[i0];
    Vector3d P1 = vertices[i1];
    Vector3d P2 = vertices[i2];

    double maxVol = -1.0;
    unsigned int idx = i0;

    for(unsigned int i = 0; i < vertices.size(); ++i)
    {
        if(i == i0 || i == i1 || i == i2) continue;

        Vector3d Pi = vertices[i];
        double vol = std::abs((P1-P0).dot((P2-P0).cross(Pi-P0))); // 6*volume
        if(vol > maxVol)
        {
            maxVol = vol;
            idx = i;
        }
    }

    return idx;
}



Mesh::Mesh(){}
Mesh::~Mesh(){}
//--
//
// ClearAll
//
//--

void Mesh::ClearAll()
{
	// Initialize mesh
	// Clear all data
	ClearVertices();
	ClearFaces();
	ClearColors();
	ClearTextures();
	ClearFaceNormals();
	ClearVertexNormals();
	ClearTextureName();
}

void Mesh::GetGeometricStatistics() const
{
// -------------------------------------------------------------------
// DIAGNOSTIC GÉOMÉTRIQUE (Avant Polissage)
// -------------------------------------------------------------------
double minArea = std::numeric_limits<double>::max();
double maxArea = 0.0;
double sumArea = 0.0;
double minEdge = std::numeric_limits<double>::max();
double maxEdge = 0.0;
double sumEdge = 0.0;
int edgeCount = 0;

for (const auto& face : faces) {
	Eigen::Vector3d v0 = vertices[face[0]];
	Eigen::Vector3d v1 = vertices[face[1]];
	Eigen::Vector3d v2 = vertices[face[2]];

	// Calcul de l'aire (Norme du produit vectoriel / 2)
	double area = 0.5 * ((v1 - v0).cross(v2 - v0)).norm();
	minArea = std::min(minArea, area);
	maxArea = std::max(maxArea, area);
	sumArea += area;

	// Calcul des longueurs d'arêtes
	double e0 = (v1 - v0).norm();
	double e1 = (v2 - v1).norm();
	double e2 = (v0 - v2).norm();

	minEdge = std::min({ minEdge, e0, e1, e2 });
	maxEdge = std::max({ maxEdge, e0, e1, e2 });
	sumEdge += (e0 + e1 + e2);
	edgeCount += 3;
}

std::cout << "\n--- MESH STATISTICS (Pre-Cleanup) ---" << std::endl;
std::cout << "Triangles : " << faces.size() << std::endl;
std::cout << "Area      : Min=" << minArea << " | Max=" << maxArea << " | Avg=" << (sumArea / faces.size()) << std::endl;
std::cout << "Edges     : Min=" << minEdge << " | Max=" << maxEdge << " | Avg=" << (sumEdge / edgeCount) << std::endl;
std::cout << "-------------------------------------\n" << std::endl;
}
//--
//
// ComputeFaceNormals
//
//--
// Compute unit normal of every faces
/**
 * @brief Calcule les normales de faces du maillage.
 * Gère les triangles dégénérés (aire nulle) pour éviter les erreurs numériques.
 */
void Mesh::ComputeFaceNormals() {
	this->face_normals.clear();
	this->face_normals.reserve(this->faces.size());

	int nullGradientCount = 0;

	for (const auto& face : this->faces) {
		// Récupération des sommets
		const Eigen::Vector3d& v0 = this->vertices[face[0]];
		const Eigen::Vector3d& v1 = this->vertices[face[1]];
		const Eigen::Vector3d& v2 = this->vertices[face[2]];

		// Calcul des arêtes du triangle
		Eigen::Vector3d edge1 = v1 - v0;
		Eigen::Vector3d edge2 = v2 - v0;

		// Produit vectoriel pour obtenir la normale brute
		Eigen::Vector3d rawNormal = edge1.cross(edge2);
		double areaSq = rawNormal.squaredNorm();

		if (areaSq > 1e-16) { // Epsilon fin pour gérer la précision double
			// Normale valide, on la normalise
			this->face_normals.push_back(rawNormal.normalized());
		}
		else {
			// Triangle dégénéré (aire quasi-nulle) !
			// La dichotomie a créé une face orpheline.
			// On attribue une normale arbitraire (ex: (1,0,0)) pour éviter le crash
			// mais ce triangle devrait idéalement être supprimé.
			this->face_normals.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
			nullGradientCount++;
		}
	}

	if (nullGradientCount > 0) {
		//std::cout << "[WARNING] Mesh::ComputeFaceNormals : Created " << nullGradientCount
			//<< " placeholder normals for degenerate faces. Orientation issue suspected." << std::endl;
	}
}

//--
//
// ComputeVertexNormals
//
//--
// Compute unit normal of every vertex
void Mesh::ComputeVertexNormals()
{

	int i;

	// Assume that face normals are computed
	assert( FaceNormalNumber() == FaceNumber() );

	// Resize and initialize vertex normal array
	vertex_normals.assign( VertexNumber(), Vector3d(0,0,0) );

	// For every face
	for( i=0 ; i<FaceNumber() ; i++ )
	{
		// Add face normal to vertex normal
		VertexNormal(i,0) += FaceNormal(i);
		VertexNormal(i,1) += FaceNormal(i);
		VertexNormal(i,2) += FaceNormal(i);
	}

	// For every vertex
	for( i=0 ; i<VertexNumber() ; i++)
	{
		// Normalize vertex normal
		VertexNormal(i).normalized();
	//	logfile<<vertex_normals[i]<<endl;
	}


}

//--
//
// ReadFile
//
//--
// Read mesh from a file
bool Mesh::ReadFile( const string& file_name )
{
	FileFormat file_format(UNKNOWN_FILE);

	cout<<"READ FILE"<<endl;
	// Find file extension
	int pos = file_name.find_last_of('.');
	if( pos == -1 )
	{
		// File has no extension
		return false;
	}

	// Format extension string
	string extension = UpperCase( file_name.substr( ++pos ) );

	cout<<"Extension="<<extension.data()<<endl;

	//Point Cloud extension
	if( extension == "XYZ" )
	{
		file_format = CLOUD_FILE;
	}
	else
	// RANGE TXT extension
	if( extension == "TXT" )
	{
		file_format = RANGE_FILE;
	}
	else
	// WRL extension
	if( extension == "WRL" )
	{
		file_format = VRML_1_FILE;
	}
	// IV extension
	else if( extension == "IV" )
	{
		file_format = IV_FILE;
	}
	// SMF extension
	else if( extension == "SMF" )
	{
		file_format = SMF_FILE;
	}
	// STL extension
	else if( extension == "STL" )
	{
		ifstream file(file_name.c_str(), ios::binary);
		if( file.is_open() == false )
		{
			return false;
		}
		char header[5];
		file.read(header, 5);
		if( strcmp(header, "solid") == 0 )
		{
			file_format = STL_ASCII_FILE;
		}
		else
		{
			file_format = STL_BINARY_FILE;
		}
		file.close();
	}
	// STLA extension
	else if( extension == "STLA" )
	{
		file_format = STL_ASCII_FILE;
	}
	// STLB extension
	else if( extension == "STLB" )
	{
		file_format = STL_BINARY_FILE;
	}
	// Other extension
	else
	{
		// Unknown file format
		cout<<"Unknown file extension"<<endl;
		return false;
	}

	// Read file
	switch( file_format )
	{

		// OpenInventor file
		case IV_FILE :

		// VRML 1.0 file
		case VRML_1_FILE :
			return ReadIv( *this, file_name );

		// Other file
		default :{cout<<"Unknown file format"<<endl;
			// Unknown file format
			return false;
				 }
	}
}

//--
//
// WriteFile
//
//--
// Write mesh to a file
bool Mesh::WriteFile( const string& file_name, const FileFormat& file_format ) const
{
	// Write file
	switch( file_format )
	{
		// OpenInventor file
		case IV_FILE :
			return WriteIv( *this, file_name, false );
			break;

		// SMF file
		case SMF_FILE :
		//	return WriteSmf( *this, file_name );
			break;

		// STL ascii file
		case STL_ASCII_FILE :
		//	return WriteStlA( *this, file_name );
			break;

		// STL binary file
		case STL_BINARY_FILE :
		//	return WriteStlB( *this, file_name );
			break;

		// VRML 1.0 file
		case VRML_1_FILE :
			return WriteIv( *this, file_name, true );
			break;

		// Other file
		default :
			// Unknown file format
			return false;
	}

    return false;
}

//--
//
// UpperCase
//
//--
// Upper case string of a given one
string UpperCase( const string& s )
{
	// Upper case string
	string us;

	// For every character in the string
	string::const_iterator it = s.begin();
	while( it != s.end() )
	{
		// Convert character to upper case
		us += toupper( *it );

		// Next character
		++it;
	}

	// Return upper case string
	return us;
}



void Mesh::Draw_Default(int i){


	for(int j=0; j<3; j++)
		glVertex3d(vertices[i][0],vertices[i][1],vertices[i][2]);


}

void Mesh::Draw_Face_Normal(int i) const{



	for(int j=0; j<3; j++)	{
		glNormal3d(face_normals[i][0],face_normals[i][1],face_normals[i][2]);
		Vector3d V=Vertex(i,j);
		glVertex3d(V[0],V[1],V[2]);
	}


}

void Mesh::Draw_Vertex_Normal(int i){



	for(int j=0; j<3; j++)	{

		int vertex_index = faces[i][j];

		Vector3d N(vertex_normals[vertex_index]);
		Vector3d V(vertices[vertex_index]);
		glNormal3d(N[0],N[1],N[2]);
		glVertex3d(V[0],V[1],V[2]);
	}


}




void Mesh::Draw(int DRAW_MODE)
{
size_t nb_faces=faces.size(),i;


switch(DRAW_MODE)
	{
	case ONLY_VERTEX	:
		{
			glDisable(GL_LIGHTING);
			cout<<"Vertices displayed="<<vertices.size()<<endl;

			glPointSize(5.0);
			glBegin(GL_POINTS);
			bool color_on=false;
			if(colors.size() == vertices.size()) color_on=true;
			for( i=0 ; i<vertices.size(); i++ )
			{
				if(color_on) glColor3f(colors[i][0],colors[i][1],colors[i][2]);
				glVertex3d(vertices[i][0],vertices[i][1],vertices[i][2]);
			}

			glEnd();

			glEnable(GL_LIGHTING);

		}break;
/*
	case FACE_NORMAL:
		{	glBegin(GL_TRIANGLES);
			for( i=0; i<nb_faces; i++) 				Draw_Face_Normal(i);
			glEnd();
		}break;
		*/
	case FACE_NORMAL:
	{
		// 1. Rendu standard de tes triangles illuminés
		glBegin(GL_TRIANGLES);
		for (i = 0; i < nb_faces; i++) {
			Draw_Face_Normal(i);
		}
		glEnd();

		/*
		// 2. RUSTINE DE DIAGNOSTIC : Dessin des vecteurs normaux physiques
		glDisable(GL_LIGHTING); // Désactive la lumière pour avoir une ligne de couleur pure
		glLineWidth(2.0f);      // Épaissit un peu la ligne pour bien la voir
		glColor3f(1.0f, 0.0f, 0.0f); // Rouge vif pour intercepter le sens du vecteur

		glBegin(GL_LINES);
		for (i = 0; i < nb_faces; i++) {
			// Extraction directe des indices depuis ton vecteur de faces global (ajuste le nom si nécessaire)
			Eigen::Vector3i F = faces[i];

			const Eigen::Vector3d& p0 = vertices[F.x()];
			const Eigen::Vector3d& p1 = vertices[F.y()];
			const Eigen::Vector3d& p2 = vertices[F.z()];

			// Calcul du barycentre de la face
			Eigen::Vector3d barycenter = (p0 + p1 + p2) / 3.0;

			// Calcul de la normale géométrique brute issue de l'indexation (CCW)
			Eigen::Vector3d normal = (p1 - p0).cross(p2 - p0);
			if (normal.isZero(1e-12)) continue;
			normal.normalize();

			// Longueur du vecteur adaptée à la taille de tes cylindres (ex: 0.05)
			double lineLength = 0.05;
			Eigen::Vector3d normalEnd = barycenter + lineLength * normal;

			// Dessin du segment
			glVertex3d(barycenter.x(), barycenter.y(), barycenter.z());
			glVertex3d(normalEnd.x(), normalEnd.y(), normalEnd.z());
		}
		glEnd();

		glEnable(GL_LIGHTING); // On réactive la lumière pour la suite du rendu
		glLineWidth(1.0f);     // On reset l'épaisseur des lignes
		*/
	}
	break;

	case VERTEX_NORMAL		:
		{
			glBegin(GL_TRIANGLES);
			for( i=0; i<nb_faces; i++) Draw_Vertex_Normal(i);
			glEnd();
		}	break;

	case VERTEX_NORMAL_RGB	:
		{
			glBegin(GL_TRIANGLES);
			for( i=0; i<faces.size(); i++) Draw_Vertex_Normal_Rgb(i);
			glEnd();
		}	break;

	}


}


void Mesh::Print_Stats()
{
	char s[250];
    //sprintf(s,"Faces   : %d", int (faces.size()));
	//PrintMessage( 10, 10, s );
    //sprintf(s,"Vertices: %d", int (vertices.size()));
	//PrintMessage( 10, 25, s );

	//add any information you want to display as text in the OGL window here

}


void Mesh::Draw_Face_Normal_Rgb(int i)
{


	for(int j=0; j<3; j++)	{
		glNormal3d(face_normals[i][0],face_normals[i][1],face_normals[i][2]);
		Eigen::Vector3d V=vertices[faces[i][j]];
		Eigen::Vector3d C=colors[faces[i][j]];
		glColor3d(C[0],C[1],C[2]);
		glVertex3d(V[0],V[1],V[2]);
	}


}

void Mesh::Draw_Vertex_Normal_Rgb(int i){

	for(int j=0; j<3; j++)	{

		int vertex_index = faces[i][j];

		Eigen::Vector3d N(vertex_normals[vertex_index]);
		Eigen::Vector3d V(vertices[vertex_index]);
		Eigen::Vector3d Col(colors[vertex_index]);

		glNormal3d( N[0] , N[1] , N[2]);
		glColor3d( Col[0] , Col[1] , Col[2]);
		glVertex3d( V[0] , V[1] , V[2]);
	}


}


unsigned int Mesh::computeBoundingSphere(Eigen::Vector3d &center, double &radius) const
{
    // 1. Centre approximatif
    center.setZero();
    for(const auto& v : vertices) center += v;
    center /= vertices.size();

    // 2. Chercher le point le plus éloigné
    unsigned int idxMax = 0;
    double maxDist2 = (vertices[0] - center).squaredNorm();
    for(unsigned int i = 1; i < vertices.size(); ++i)
    {
        double d2 = (vertices[i] - center).squaredNorm();
        if(d2 > maxDist2)
        {
            idxMax = i;
            maxDist2 = d2;
        }
    }

    // 3. Rayon
    radius = std::sqrt(maxDist2);

    return idxMax;
}

unsigned int Mesh::findClosestPointToPlane(unsigned int i0, const Eigen::Vector3d &N) const
{
    const double eps = 1e-12;
    unsigned int idxClosest = i0; // fallback
    double minDist = std::numeric_limits<double>::max();

	Eigen::Vector3d P0 = vertices[i0];

    for(unsigned int i = 0; i < vertices.size(); ++i)
    {
        if(i == i0) continue; // éviter le point dégénéré
        double d = N.dot(vertices[i] - P0); // distance signée au plan
        if(d > eps && d < minDist) // côté “extérieur” et le plus proche
        {
            minDist = d;
            idxClosest = i;
        }
    }

    // fallback si aucun point trouvé (rare)
    if(idxClosest == i0)
    {
        for(unsigned int i = 0; i < vertices.size(); ++i)
        {
            if(i != i0)
            {
                idxClosest = i;
                break;
            }
        }
    }

    return idxClosest;
}

unsigned int Mesh::findThirdPointMinimalRotation(unsigned int i0, unsigned int i1, const Vector3d &N) const
{
    const double eps = 1e-12;
    Vector3d P0 = vertices[i0];
    Vector3d P1 = vertices[i1];
    Vector3d u = (P1 - P0).normalized(); // axe P0->P1

    unsigned int i2 = i0; // fallback
    double minAngleAbs = std::numeric_limits<double>::max();

    for(unsigned int i = 0; i < vertices.size(); ++i)
    {
        if(i == i0 || i == i1) continue;

        Vector3d v = vertices[i] - P0;
        Vector3d vProj = v - (v.dot(u)) * u; // projection orthogonale à l'axe
        double normProj = vProj.norm();

        if(normProj < eps) continue; // point quasi aligné sur l'axe ? plan dégénéré

        // angle minimal nécessaire pour inclure Pi
        double theta = std::asin( (N.dot(v)) / normProj );

        if(std::abs(theta) < minAngleAbs)
        {
            minAngleAbs = std::abs(theta);
            i2 = i;
        }
    }

    // fallback si aucun point valide trouvé
    if(i2 == i0 || i2 == i1)
    {
        for(unsigned int i = 0; i < vertices.size(); ++i)
        {
            if(i != i0 && i != i1)
            {
                i2 = i;
                break;
            }
        }
    }

    return i2;
}


unsigned int Mesh::correctThirdPoint(unsigned int i0, unsigned int i1, unsigned int i2)
{
    const double eps = 1e-12;
    Vector3d P0 = vertices[i0];
    Vector3d P1 = vertices[i1];
    unsigned int current_i2 = i2;

    while(true)
    {
        Vector3d P2 = vertices[current_i2];
        Vector3d N = (P1 - P0).cross(P2 - P0).normalized();

        double minD = 0.0;
        unsigned int iWorst = current_i2;

        for(unsigned int i = 0; i < vertices.size(); ++i)
        {
            if(i == i0 || i == i1 || i == current_i2) continue;

            double d = N.dot(vertices[i] - P0);

            if(d < minD) // point du mauvais côté le plus extrême
            {
                minD = d;
                iWorst = i;
            }
        }

        if(iWorst == current_i2 || std::abs(minD) < eps)
            break; // tous les points sont du bon côté

        current_i2 = iWorst; // remplacer i2 par le point le plus “négatif”
    }

    return current_i2;
}




void Mesh::PurgeInternalFaces(double epsilon_surf) {
	// Utilisation de std::remove_if pour identifier les faces à supprimer
	faces.erase(std::remove_if(faces.begin(), faces.end(), [&](const Eigen::Vector3i& face) {

		int posCount = 0;
		int idx[3] = { face.x(), face.y(), face.z() };

		for (int j = 0; j < 3; ++j) {
			// On vérifie la valeur ternaire stockée pour chaque sommet de la face
			if (vertex_results[idx[j]].f_tern > epsilon_surf) {
				posCount++;
			}
		}

		// Si posCount == 3, la face est totalement intérieure -> on retourne true pour supprimer
		return (posCount == 3);

		}), faces.end());
}