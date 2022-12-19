#pragma once
#include "ParticleLink.h"
class ParticleRod : public ParticleLink
{
public:
	float length;

	ParticleContact* createContact();
};

