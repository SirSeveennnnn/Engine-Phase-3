#include "ParticleContact.h"

void ParticleContact::resolve(float duration)
{
	resolveVelocity(duration);
	resolveInterpenetration(duration);
}

float ParticleContact::calculateSeparatingVelocity()
{
	
	glm::vec3 relativeVelocity = particle[0]->GetVelocity();
	if (particle[1] != nullptr)
	{
		relativeVelocity -= particle[1]->GetVelocity();
	}

	return (relativeVelocity.x * contactNormal.x) + (relativeVelocity.y * contactNormal.y) + (relativeVelocity.z * contactNormal.z);
}

void ParticleContact::resolveVelocity(float duration)
{
	float separatingVelocity = calculateSeparatingVelocity();

	//cout << "Separating velocity: " << separatingVelocity << endl;

	if (separatingVelocity > 0)
	{
		return;
	}
	//cout << "Passthrough" << endl;

	float newSeparatingVelocity = -separatingVelocity * restitution;
	float deltaVelocity = newSeparatingVelocity - separatingVelocity;

	float totalInverseMass = particle[0]->GetInverseMass();
	if (particle[1] != nullptr)
	{
		totalInverseMass += particle[1]->GetInverseMass();
	}

	if (totalInverseMass <= 0)
	{
		return;
	}

	float impulse = deltaVelocity / totalInverseMass; //replaced / with *

	glm::vec3 impulsePerIMass = contactNormal * impulse;
	particle[0]->SetVelocity(particle[0]->GetVelocity() + impulsePerIMass * particle[0]->GetInverseMass());

	if (particle[1] != nullptr)
	{
		particle[1]->SetVelocity(particle[1]->GetVelocity() + impulsePerIMass * -particle[1]->GetInverseMass());
	}
	
}

void ParticleContact::resolveInterpenetration(float duration)
{
	if (penetration <= 0)
	{
		return;
	}

	float totalInverseMass = (1 / particle[0]->GetInverseMass());
	if (particle[1] != nullptr)
	{
		totalInverseMass += (1 / particle[1]->GetInverseMass());
	}

	if (totalInverseMass <= 0)
	{
		return;
	}

	glm::vec3 movePerIMass = contactNormal * (penetration / totalInverseMass); //original was -penetration

	glm::vec3 newPosA = movePerIMass * particle[0]->GetInverseMass();
	particle[0]->SetPosition(particle[0]->GetPosition() + newPosA);

	if (particle[1] != nullptr)
	{
		glm::vec3 newPosB = movePerIMass * -particle[1]->GetInverseMass();
		particle[1]->SetPosition(particle[1]->GetPosition() + newPosB);
	}

	penetration = 0;

}
