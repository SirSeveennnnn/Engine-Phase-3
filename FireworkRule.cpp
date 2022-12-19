#include "FireworkRule.h"

//Put Tables in Velocity Table
FireworkRule::FireworkRule()
{
	velocityList.push_back(glm::vec3(10, 20, 15));
	velocityList.push_back(glm::vec3(25, 25, -15));
	velocityList.push_back(glm::vec3(-28, -20, -30));
	velocityList.push_back(glm::vec3(-15, -10, 10));
	velocityList.push_back(glm::vec3(-20, 30, 10));
}

//Get values based on rules
void FireworkRule::SetRules(FireworkType fType, int index)
{
	switch (fType)
	{
	case Parent:
		type = fType;
		minAge = 1;
		maxAge = 3;
		velocity = glm::vec3(20, 30, -10);
		dampingMult = 0.999f;
		break;
	case Child:
		type = fType;
		minAge = 1;
		maxAge = 2;
		velocity = velocityList[index];
		dampingMult = 0.99f;
		break;
	default:
		break;
	}
}
