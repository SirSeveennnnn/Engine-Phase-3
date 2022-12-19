#include "ElasticBungeeForce.h"

ElasticBungeeForce::ElasticBungeeForce(Particle* other, float springConstant, float restLength)
{
	this->other = other;
	this->springConstant = springConstant;
	this->restLength = restLength;
}

void ElasticBungeeForce::updateForce(Particle* particle, float time)
{
	glm::vec3 force = particle->GetPosition();
	force -= other->GetPosition();

	float magnitude = sqrt((force.x * force.x) + (force.y * force.y) + (force.z * force.z));
	if (magnitude <= restLength)
	{
		return;
	}
	magnitude = springConstant * (restLength - magnitude);

	glm::normalize(force); 
	force *= magnitude;

	particle->AddForce(force);

	//cout << "Magnitude: " << magnitude << endl;
	//cout << "pos: x:" << pos.x << " y:" << pos.y << " z:" << pos.z << endl;
	//cout << "otherpos: x:" << otherpos.x << " y:" << otherpos.y << " z:" << otherpos.z << endl;
	//cout << "force: x:" << force.x << " y:" << force.y << " z:" << force.z << endl;
}
