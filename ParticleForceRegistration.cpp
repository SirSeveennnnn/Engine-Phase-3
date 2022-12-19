#include "ParticleForceRegistration.h"

ParticleForceRegistration::ParticleForceRegistration(Particle* p, ParticleForceGenerator* fg)
{
	this->particle = p;
	this->forceGenerator = fg;
}
