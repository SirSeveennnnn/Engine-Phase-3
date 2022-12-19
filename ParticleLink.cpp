#include "ParticleLink.h"

float ParticleLink::currentLength()
{
	glm::vec3 distance = particle[0]->GetPosition() - particle[1]->GetPosition();
	return glm::length(distance);
}
