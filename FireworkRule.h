#pragma once
#include<glad/glad.h>
#include <GLFW/glfw3.h>

#include<glm/glm.hpp>
#include<glm/gtc/matrix_transform.hpp>
#include<glm/gtc/type_ptr.hpp>


using namespace std;
#include<string>
#include<vector>
#include<iostream>

//Firework Types
enum FireworkType {
	Parent,
	Child
};

class FireworkRule
{
public:
	FireworkRule();
	void SetRules(FireworkType fType, int index);

	//Type of Firworks it will be
	FireworkType type;

	//Minimum Age of the Firework
	float minAge;
	//Maxumum Age of the Firework
	float maxAge;

	//The velocity of the firework
	glm::vec3 velocity;

	/*
	* Good day sir, 
	* about the velocity, i tried to random the velocity using srand() and rand() in C++ 
	* but it keeps giving me the same values per call of the rand() function.
	* ex. 5 firework have 1,3,4 velocity.
	* 
	* which is why i implemented a velocity table that gives different velocities to the fireworks
	*/
	vector<glm::vec3> velocityList;
	
	//Damping multiplier
	float dampingMult;

	//amount of payload of the firework
	int payloadCount = 5;

};

