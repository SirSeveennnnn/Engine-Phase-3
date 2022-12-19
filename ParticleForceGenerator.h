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

#include<unordered_map>
#include<string>

using namespace std;
class ParticleForceGenerator
{
public:
	virtual void updateForce(Particle* particle, float time) = 0;
	string name = "force";

};

