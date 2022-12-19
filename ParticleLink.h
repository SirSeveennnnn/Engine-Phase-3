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
#include"ParticleContact.h"
#include"ParticleContactResolver.h"

#include"MyShaders.h"
#include"Camera.h"

#include<unordered_map>
#include<string>

class ParticleLink
{
public:
	Particle* particle[2];

protected:
	float currentLength();
};

