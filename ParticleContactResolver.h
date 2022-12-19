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

#include<unordered_map>
#include<string>

class ParticleContactResolver
{
public:
	float maxIterations = 0;
	ParticleContactResolver(int maxIt);
	void ResolveContacts(vector<ParticleContact*> contacts, float duration);

protected:
	
	float currentIterations = 0;

};

