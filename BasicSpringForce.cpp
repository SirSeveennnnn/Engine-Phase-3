#include "BasicSpringForce.h"

BasicSpringForce::BasicSpringForce(Particle* other, float springConstant, float restLength)
{
	this->other = other;
	this->springConstant = springConstant;
	this->restLength = restLength;

}

void BasicSpringForce::updateForce(Particle* particle, float time)
{

	glm::vec3 force = particle->GetPosition();
	force -= other->GetPosition();

	float magnitude = sqrt((force.x * force.x) + (force.y * force.y) + (force.z * force.z));
	magnitude = abs(magnitude - restLength);
	magnitude *= springConstant;

	glm::normalize(force);
	force *= -magnitude;
	particle->AddForce(force);
	
}
