#pragma once
#include "Particle.h"
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <random>

#include"FireworkRule.h"

class Fireworks : public Particle
{
public:
	//Update function
	Fireworks(glm::vec3 point);
	~Fireworks();
	void Update(float time, vector<Fireworks*>& fireworksList, int& firewordIndex, int fireworkLimit);

	//Create with the fireworks rule
	void FireFireworks(FireworkType type, int velocityTableIndex, Fireworks* parent = nullptr);


private:
	FireworkRule rule;
	//Age (lifespan) is already in particle class
};

