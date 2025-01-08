/*
Non-Commercial License - Mississippi State University Autonomous Vehicle Software (MAVS)

ACKNOWLEDGEMENT:
Mississippi State University, Center for Advanced Vehicular Systems (CAVS)

*NOTICE*
Thank you for your interest in MAVS, we hope it serves you well!  We have a few small requests to ask of you that will help us continue to create innovative platforms:
-To share MAVS with a friend, please ask them to visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to register and download MAVS for free.
-Feel free to create derivative works for non-commercial purposes, i.e. academics, U.S. government, and industry research.  If commercial uses are intended, please contact us for a commercial license at otm@msstate.edu or jonathan.rudd@msstate.edu.
-Finally, please use the ACKNOWLEDGEMENT above in your derivative works.  This helps us both!

Please visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to view the full license, or email us if you have any questions.

Copyright 2018 (C) Mississippi State University
*/
/**
 * \file mesh.h
 *
 * Default mesh format for MAVS scenes is wavefront obj. This file contains
 * methods for loading, storing, and manipulating mesh objects.
 *
 * \author Chris Goodin
 *
 * \date 12/18/2017
 */

#ifndef MESH_H
#define MESH_H

#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <raytracers/material.h>
#include <raytracers/bounding_box.h>

#include <iostream>

namespace mavs {
namespace raytracer {

/// Triangular face structure
struct Face {
	int v1;
	int v2;
	int v3;
	int t1;
	int t2;
	int t3;
	int n1;
	int n2;
	int n3;
	glm::vec3 normal;
	//std::string group;
	char group[256];
};

/**
	* The Mesh class loads a wavefront obj mesh and stores the faces, vertices,
	* and materials. Textures are stored at a scene level so that shared textures
	* are not loaded twice.
	*/
class Mesh {
public:
	Mesh();
	~Mesh();

	/**
		* Loads a wavefront obj mesh. The material file and texture files should be
		* in the same directory as the .obj file.
		* \param inpath File path to the directory containing the obj and mtl files.
		* \param fname File name of the obj file to load.
		*/
	void Load(std::string inpath, std::string fname);

	/**
	* Loads a wavefront obj mesh. The material file and texture files should be
	* in the same directory as the .obj file. Specify the mtl file to use, 
	* overriding the one specified in the .obj file
	* \param inpath File path to the directory containing the obj and mtl files.
	* \param fname File name of the obj file to load.
	* \param mtllib_filename The name of the .mtl file to use
	*/
	void Load(std::string inpath, std::string fname, std::string mtllib_filename);

	/**
	* Write a binary obj file. Full path with .bin extension
	* \param binary_out The name of the binary file to write out, with full path and .bin extension
	*/
	void WriteBinary(std::string inpath, std::string binary_out);

	/**
	* Read a binary obj file. Full path with .bin extension
	* \param binary_in The name of the binary file to read in, with full path and .bin extension
	*/
	void ReadBinary(std::string inpath, std::string binary_in);

	/**
		* Write the mesh to an obj file
		* \param ofname The output .obj file name, WITHOUT extension
		*/
	void Write(std::string ofname);

	/// Returns a vector list of all the textures used by the material file.
	std::vector<std::string> GetAllTextureNames();

	/**
	* Load an obj file line by line and transform the vertices
	* using an affine transform
	* \param infile The file to load
	* \param outfile The file to save
	* \param aff_rot The affine rotation to apply
	*/
	void LoadTransformAndSave(std::string infile, std::string outfile, glm::mat3x4 aff_rot);

	/// Returns the name of the mesh, typically the .obj file name.
	std::string GetName() { return mesh_name_; }

	/// Returns the number of triangular facets in the mesh.
	size_t GetNumFaces()const { return faces_.size(); }

	/// Returns the number of vertices in the mesh.
	size_t GetNumVertices()const { return vertices_.size(); }

	/// Retruns the number of texture coordinates (vt) in the mesh
	size_t GetNumTexCoords()const { return texcoords_.size(); }

	void SetNumTexCoords(int numtex) { texcoords_.resize(numtex); }

	///Returns the number of materials
	size_t GetNumMaterials() const { return materials_.size(); }

	/// Calculates the spatial dimensions of the mesh to stdout.
	glm::vec3 GetSize();

	/// Returns the spatial coordinates of the center of the mesh bounding box.
	glm::vec3 GetCenter();

	/// Add a material to the list of materials for the object
	void AddMaterial(Material &mat) { materials_.push_back(mat); }

	/**
		* Translates the mesh by "offset" meters.
		* \param offset Translation vector of the mesh in meters.
		*/
	void Translate(glm::vec3 offset);

	/**
		* Scales the mesh independently in the x, y, and z directions
		*/
	void Scale(float sx, float sy, float sz);

	/**
		* Rotates the mesh by 90 degrees about the x-axis such that the y axis
		* becomes z and z becomes -y.
		*/
	void RotateYToZ();

	/**
	* Rotates the mesh by -90 degrees about the x-axis such that the y axis
	* becomes -z and z becomes y.
	*/
	void RotateZToY();

	/**
	* Rotates the mesh by 90 degrees about the z-axis such that the x axis
	* becomes y and y becomes -x. Equivalent to a counterclockwise rotation.
	*/
	void RotateXToY();

	/**
	* Rotates the mesh by 90 degrees about the z-axis such that the y axis
	* becomes x and x becomes -y. Equivalent to a clockwise rotation.
	*/
	void RotateYToX();

	/// Get the face [i]
	Face GetFace(int i) { return faces_[i]; }

	/// Get the vertex [i]
	glm::vec3 GetVertex(int i) { return vertices_[i]; }

	/// Get the normal [i]
	glm::vec3 GetNormal(int i) { return normals_[i]; }

	/**
	* Set the normal of the ith vertex
	* \param i The normal number to set
	* \param normal The normal to set, should be already normalized
	*/
	void SetNormal(int i, glm::vec3 normal) {
		if (i>=0 && i<normals_.size()) normals_[i] = normal;
	}

	/// Get the texcoord [i]
	glm::vec2 GetTexCoord(int i) { return texcoords_[i]; }

	void ClearTexCoords() { texcoords_.clear(); }

	int AddTexCoord(float u, float v) {
		texcoords_.push_back(glm::vec2(u, v));
		return (int)texcoords_.size() + 1;
	}

	void SetFaceTextureIndex(int facenum, int t1, int t2, int t3) {
		if (facenum >= 0 && facenum < faces_.size()) {
			faces_[facenum].t1 = t1;
			faces_[facenum].t2 = t2;
			faces_[facenum].t3 = t3;
		}
	}

	/**
	* Set the coordinates of a texture vertex
	* \param i Number of the texture vertex
	* \param u Normalized x-coordinate of the texture
	* \param v Normalized y-coordinate of the texture
	*/
	void SetTexCoord(int i, float u, float v) {
		if (i >= 0 && i < texcoords_.size()) {
			texcoords_[i].x = u;
			texcoords_[i].y = v;
		}
	}

	/**
		* Return the ith material
		* \param i The number of the material to get
		*/
	Material * GetMaterial(int i) { return &materials_[i]; }

	/**
		* Return the ith material
		* \param i The number of the material to get
		*/
	void GetMaterial(int i, Material &material) { material = materials_[i]; }

	/**
		* Return the number of the ith material
		* \param i The number of the material to get
		*/
	int GetMatNum(int i) {
		return mat_nums_[i];
	}

	/**
	* Apply an affine translation, rotation, and scale to the mesh
	* \param aff_rot The 3x4 rotation matrix
	*/
	void ApplyAffineTransformation(glm::mat3x4 aff_rot);

	///Method to clear geometry but keep material info
	void ClearMemory() {
		faces_.clear();
		vertices_.clear();
		normals_.clear();
	}

	/// Return the axis-aligned bounding box of the mesh 
	BoundingBox GetBoundingBox() {
		if (!size_calculated_) {
			glm::vec3 size = GetSize();
		}
		BoundingBox bb(min_corner_, max_corner_);
		return bb;
	}

	/**
	* Get mesh height at a given location. Will
	* take the highest point of the mesh
	* \param position The point to take the height
	*/
	float GetMeshHeight(glm::vec2 position);

	/// Return the orientation of the mesh as a quaternion
	glm::quat GetOrientation() { return orientation_; }

	/**
	* Set the orientation of the mesh as a quaternion
	* \param ori The new orientation of the mesh
	*/
	void SetOrientation(glm::quat ori) { orientation_ = ori; }

	/**
	* Call this to label the mesh by group rather than at a mesh level
	* \param lbg Set to true to label by group. Default if false.
	*/
	void SetLabelByGroup(bool lbg) {
		label_by_group_ = lbg;
	}

	/// Returns wether the mesh is labeled by group or not
	bool GetLabelByGroup() {
		return label_by_group_;
	}

	/**
	* Return the group name of a given triangle
	* \param face_id The id number of the triangle
	*/
	std::string GetGroupOf(int face_id) {
		std::string group("");
		if (face_id >= 0 && face_id < (int)faces_.size()) {
			group = faces_[face_id].group;
		}
		return group;
	}

	/**
	* Clear out all the properties of the mesh
	*/
	void ClearAll();

	/**
	* Calculate the normal at each vertex
	*/
	void CalculateNormals();

	int GetNumNormals() { return (int)normals_.size(); }

	/// return true if vertex normals have been calculated
	bool HasNormals() {
		if (normals_.size() == vertices_.size() && normals_.size()>0) {
			return true;
		}
		else {
			return false;
		}
	}

	void SetVelocity(glm::vec3 vel) { velocity_ = vel; }

	glm::vec3 GetVelocity() { return velocity_; }

private:
	glm::vec3 CalculateTriangleNormal(glm::vec3 p1, glm::vec3 p2, glm::vec3 p3);
	
	glm::vec3 velocity_;

	std::vector<Face> faces_;
	std::vector<int> mat_nums_;
	std::vector<glm::vec3> vertices_;
	std::vector<glm::vec3> normals_;
	std::vector<glm::vec2> texcoords_;
	std::vector<Material> materials_;
	std::string mesh_name_;

	glm::vec3 max_corner_;
	glm::vec3 min_corner_;
	glm::vec3 center_;
	glm::quat orientation_;
	bool size_calculated_;

	bool label_by_group_;
};

} //namespace raytracer
} //namespace mavs

#endif
