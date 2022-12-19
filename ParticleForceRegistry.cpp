#include "ParticleForceRegistry.h"


ParticleForceRegistry::ParticleForceRegistry()
{

}

void ParticleForceRegistry::add(Particle* particle, ParticleForceGenerator* forceGenerator)
{
	ParticleForceRegistration* registration = new ParticleForceRegistration(particle, forceGenerator);
	Registry.push_back(registration);
}

void ParticleForceRegistry::remove(Particle* particle, ParticleForceGenerator* forceGenerator)
{
	
	ParticleForceRegistration* registered = new ParticleForceRegistration(particle, forceGenerator);

	int index = -1;
	for (int i = 0; i < Registry.size(); i++)
	{
		if (Registry[i]->particle == registered->particle && Registry[i]->forceGenerator == registered->forceGenerator)
		{
			index = i;
		}
	}

	if (index != -1)
	{
		Registry.erase(Registry.begin() + index);
		Registry.shrink_to_fit();
	}
	
}

void ParticleForceRegistry::clear()
{
	Registry.clear();
}

void ParticleForceRegistry::UpdateForces(float time)
{
	for (int i = 0; i < Registry.size(); i++)
	{
		Registry[i]->forceGenerator->updateForce(Registry[i]->particle, time);
	}
}

int ParticleForceRegistry::Size()
{
	return Registry.size();
}
