#pragma once

#include<glad/glad.h>
#include <GLFW/glfw3.h>

#include<glm/glm.hpp>
#include<glm/gtc/matrix_transform.hpp>
#include<glm/gtc/type_ptr.hpp>

#include<string>
#include<vector>
#include<iostream>

#include"tiny_obj_loader.h"
#include"stb_image.h"

#include"Camera.h"
#include"Particle.h"
#include"Fireworks.h"
#include"ParticleGravity.h"
#include"ParticleForceRegistry.h"
#include"BasicSpringForce.h"
#include"AnchoredSpringForce.h"
#include"ElasticBungeeForce.h"
#include"ParticleContact.h"
#include"ParticleContactResolver.h"
#include"ParticleRod.h"

#include"RigidBody.h"

#include"MyShaders.h"
#include"Camera.h"

#include<unordered_map>
#include<string>

class ParticleWorld
{
public:
	//Particles
	vector<Particle*> particles;
	int particleLimit = 30;
	int particleCounter = 1; // index 0 will be a place holder

	vector<Particle*> boxParticles;
	vector<ParticleRod*> rods;

	ParticleForceRegistry registry;
	vector<ParticleContact*> contacts;

	RigidBody* rb;

	void startFrame();
	void runPhysics(float deltaTime);
	
	float Distance(glm::vec3 a, glm::vec3 b);
	void DrawUpdate(MyShaders* shader, Camera camera);
	void initialize();
	
	void FreeMemory();

protected:

	ParticleContactResolver* contactResolver = new ParticleContactResolver(10);
	void integrate(float deltaTime);
	void addContacts(Particle* a, Particle* b);
	void generateContacts();
	bool ParticleCubeCollision(Particle* particle, RigidBody* rb);

	
	
};

