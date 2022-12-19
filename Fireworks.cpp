#include "Fireworks.h"

//Fireworks constructor
Fireworks::Fireworks(glm::vec3 point) :Particle(point)
{
	/*
	SetSpawnPoint(point);
	LoadModel();
	LoadVaoVboEbo();
	LoadTransform();
	*/
}

Fireworks::~Fireworks()
{
	
}

//Update Function for the functions
void Fireworks::Update(float deltaTime, vector<Fireworks*> &fireworksList, int& fireworkIndex, int fireworkLimit)
{
	
	//Check if the fireworks is used
	if (isUsed)
	{
		//Add time to current lifetime
		currentLifeTime += deltaTime;

		//check if current lifetime is equal or greater to the assign lifetime
		if (currentLifeTime >= lifeTime)
		{
			//check if the firework is a parent
			if (this->rule.type == Parent)
			{
				
				//if it is, fire the payload
				for (int i = 0; i < 5; i++)
				{
					Fireworks* clone = fireworksList[fireworkIndex];
					clone->FireFireworks(Child, i, this);
					fireworkIndex++;
					if (fireworkIndex > fireworkLimit - 1)
					{
						fireworkIndex = 0;
					}
				}

			}
			
			//Reset the current firework
			ResetParticle();
			
		}

		//If fireworks is being used, update physics
		integrate(deltaTime);
	}
	
}

//Fire fireworks
void Fireworks::FireFireworks(FireworkType type, int velocityTableIndex,  Fireworks* parent)
{
	//Set Rules based on the firework type
	if (type == Parent)
	{
		//Set the parent rules
		rule.SetRules(type, 0);
	}
	else if (type == Child)
	{
		//Set the Child Rules
		rule.SetRules(type, velocityTableIndex);
	}

	//Set using firework
	isUsed = true;

	//Adjust min and max to C++ rand
	int minLifetime = rule.minAge;
	int maxLifetime = abs(rule.maxAge) + abs(rule.minAge);
	this->lifeTime = (rand() % maxLifetime) + minLifetime;

	//Calculate Velocity
	glm::vec3 resultVelocity = glm::vec3(0,0,0);

	//Check if firework has parent
	if (parent != nullptr)
	{
		//if firework has parent, 
		this->position = parent->GetPosition();
		SetPosition(position);

		resultVelocity = parent->GetVelocity();
	}
	else
	{
		SetPosition(spawnPoint);
	}

	//add the velocity from the rule
	resultVelocity += rule.velocity;

	//Set the physics values
	this->SetVelocity(resultVelocity);
	this->SetInverseMass(1);
	this->SetDamping(rule.dampingMult);
	this->SetAcceleration(glm::vec3(0, -20.f, 0));

}



