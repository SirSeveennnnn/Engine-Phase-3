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
#include "RigidBody.h"

#include<unordered_map>
#include<string>

class ParticleContact
{
public:
	Particle* particle[2];

	Particle* soloParticle;
	RigidBody* rb;

	float restitution;

	glm::vec3 contactNormal;
	float penetration;

	void resolve(float duration);
	float calculateSeparatingVelocity();


private:
	void resolveVelocity(float duration);
	void resolveInterpenetration(float duration);

};

