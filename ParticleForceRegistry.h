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
#include<unordered_map>
#include <algorithm>
#include"Particle.h"
#include"ParticleForceGenerator.h"
#include"ParticleForceRegistration.h"

using namespace std;

class ParticleForceRegistry
{
protected:

	vector<ParticleForceRegistration*> Registry;

public:
	ParticleForceRegistry();

	void add(Particle* particle, ParticleForceGenerator* forceGenerator);
	void remove(Particle* particle, ParticleForceGenerator* forceGenerator);
	void clear();
	void UpdateForces(float time);

	int Size();
};

