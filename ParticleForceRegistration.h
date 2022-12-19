#pragma once
#include"Particle.h"
#include "ParticleForceGenerator.h"

class ParticleForceRegistration
{
public: 
	ParticleForceRegistration(Particle* p, ParticleForceGenerator* fg);

	Particle* particle;
	ParticleForceGenerator* forceGenerator;

};

