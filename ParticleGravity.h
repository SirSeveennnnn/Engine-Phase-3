#pragma once
#include "ParticleForceGenerator.h"
#include<glad/glad.h>
#include <GLFW/glfw3.h>

#include<glm/glm.hpp>
#include<glm/gtc/matrix_transform.hpp>
#include<glm/gtc/type_ptr.hpp>

#include<string>
#include<vector>
#include<iostream>

#include"Particle.h"

#include<unordered_map>

class ParticleGravity : public ParticleForceGenerator
{
	glm::vec3 gravity = glm::vec3(0, -10.f, 0);

public:
	ParticleGravity(glm::vec3 gravityValue);
	void updateForce(Particle* particle, float time);
};

