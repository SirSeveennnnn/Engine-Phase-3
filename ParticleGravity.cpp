#include "ParticleGravity.h"

ParticleGravity::ParticleGravity(glm::vec3 gravityValue) : ParticleForceGenerator()
{
	this->gravity = gravityValue;
}


void ParticleGravity::updateForce(Particle* particle, float time)
{
	//add force
	particle->AddForce(gravity);
}
