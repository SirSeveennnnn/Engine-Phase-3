#include "ParticleRod.h"

ParticleContact* ParticleRod::createContact()
{
	float currLength = currentLength();
	if (currLength == length)
	{

		return nullptr;
	}

	//cout << "current length: " << currLength << endl;
	//cout << "length: " << length << endl;

	ParticleContact* contact = new ParticleContact();
	contact->particle[0] = particle[0];
	contact->particle[1] = particle[1];

	glm::vec3 normal = particle[1]->GetPosition() - particle[0]->GetPosition();
	float magnitude = glm::length(normal);

	normal.x = normal.x / magnitude;
	normal.y = normal.y / magnitude;
	normal.z = normal.z / magnitude;

	if (currLength > length)
	{
		contact->contactNormal = normal;
		contact->penetration = currLength - length;


	}
	else if (currLength < length)
	{

		contact->contactNormal = normal * -1.f;
		contact->penetration = length - currLength;
	}

	contact->restitution = 0;

	return contact;

}
