#pragma once

#include<glad/glad.h>
#include <GLFW/glfw3.h>

#include<glm/glm.hpp>
#include<glm/gtc/matrix_transform.hpp>
#include<glm/gtc/type_ptr.hpp>
#include<glm/gtx/quaternion.hpp>
#include<glm/gtc/quaternion.hpp>

#include<string>
#include<vector>
#include<iostream>

#include"tiny_obj_loader.h"


#include"Camera.h"


#include<unordered_map>

class RigidBody
{
public:
    RigidBody();
	void SetSpawnPoint(glm::vec3 point);
	
	// Mass of RigidBody
	float mass;

	//Position of Rb
	glm::vec3 position;

	//Quaternion Rotation of Rb
	glm::quat quaternion;

	//Euler Rotation of rb
	glm::vec3 rotation = glm::vec3(0, 0, 0);
	glm::vec3 orientation = glm::vec3(0, 45, 0);

	//Scale
	glm::vec3 scale = glm::vec3(5, 5, 5);

	//velocity of Rb
	glm::vec3 velocity;

	//acceleration of Rb
	glm::vec3 acceleration;

	//Force Accums
	glm::vec3 forceAccum;
	glm::vec3 torqueAccum;

	//Drag
	float linearDamp = 0.99f;
	float angularDamp = 0.999f;


	//Inverse Inertia Tensor
	glm::mat3 inverseInertiaTensor;

	//forceAcumm
	void addForce(glm::vec3 force);
	void addTorque(glm::vec3 point, glm::vec3 force);
	void clearAccumulators();

	//Set Inverse Inertia Tensor
	void setInverseInertiaTensor();

	void CalculateDerivedData();
	void CalculateEulerAngle();
	void Draw(GLuint bunnyShaderProgram, Camera camera);
	void Update(float deltaTime);
	void integrate(float deltaTime);

	void ResetBox();


private:
	void LoadModel();
	void LoadVaoVboEbo();
	void LoadTransform();

	//Model Variables
	std::string bunnyPath = "Models/box.obj";
	std::vector<tinyobj::shape_t> sphereShapes;
	std::vector<tinyobj::material_t> bunnyMaterial;
	std::string warning, error;
	tinyobj::attrib_t bunnyAttributes;
	bool bunnySuccess = tinyobj::LoadObj(&bunnyAttributes, &sphereShapes, &bunnyMaterial, &warning, &error, bunnyPath.c_str());
	std::vector<GLuint> sphereMesh_indices;
	GLuint bVAO, bVBO, bEBO;


	//identity matrix
	glm::mat4 identity_matrix4 = glm::mat4(1.0f);
	//Transformation Matrix of the Particle
	glm::mat4 transform;

	glm::vec3 spawnPoint = glm::vec3(0, -30.f, -50.f);
	
};

