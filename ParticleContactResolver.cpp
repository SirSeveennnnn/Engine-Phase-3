#include "ParticleContactResolver.h"

ParticleContactResolver::ParticleContactResolver(int maxIt)
{
	//this->maxIterations = maxIt;
}

void ParticleContactResolver::ResolveContacts(vector<ParticleContact*> contacts, float duration)
{

	currentIterations = 0;
	
	while (currentIterations < maxIterations)
	{
		float min = contacts[0]->calculateSeparatingVelocity();
		int minIndex = 0;
		bool resolvedAll = true;

		for (int i = 0; i < contacts.size(); i++)
		{
			float sepVelo = contacts[i]->calculateSeparatingVelocity();
			if (sepVelo < min)
			{
				resolvedAll = false;
				min = sepVelo;
				minIndex = i;
			}
		}

		contacts[minIndex]->resolve(duration);
		currentIterations++;

		if (resolvedAll)
		{
			return;
		}
	}
	
}
