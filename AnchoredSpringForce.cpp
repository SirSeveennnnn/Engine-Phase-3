#include "AnchoredSpringForce.h"

AnchoredSpringForce::AnchoredSpringForce(glm::vec3 anchorPoint, float springConstant, float restLength)
{
	this->anchor = anchorPoint;
	this->springConstant = springConstant;
	this->restLength = restLength;
}

void AnchoredSpringForce::updateForce(Particle* particle, float time)
{
	glm::vec3 force = particle->GetPosition();
	force -= anchor;

	float magnitude = sqrt((force.x * force.x) + (force.y * force.y) + (force.z * force.z));
	magnitude = abs(magnitude - restLength);
	magnitude *= springConstant;

	glm::normalize(force);
	force *= -magnitude;
	particle->AddForce(force);
}
