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
class AnchoredSpringForce : public ParticleForceGenerator
{
	glm::vec3 anchor;
	float springConstant;
	float restLength;

public: 
	AnchoredSpringForce(glm::vec3 anchorPoint , float springConstant, float restLength);
	void updateForce(Particle* particle, float time);
};

