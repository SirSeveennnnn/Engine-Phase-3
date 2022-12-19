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
class DebugManager
{
public:
	void PrintVector(string name, glm::vec3 vector3);
};

