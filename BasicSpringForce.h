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

class BasicSpringForce : public ParticleForceGenerator
{
	Particle* other;
	float springConstant;
	float restLength;

public:
	BasicSpringForce(Particle* other, float springConstant, float restLength);
	void updateForce(Particle* particle, float time);
};

