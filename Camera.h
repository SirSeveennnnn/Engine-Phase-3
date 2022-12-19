#pragma once

#include<glad/glad.h>
#include <GLFW/glfw3.h>

#include<glm/glm.hpp>
#include<glm/gtc/matrix_transform.hpp>
#include<glm/gtc/type_ptr.hpp>

#include<string>
#include<vector>
#include<iostream>


#include<unordered_map>

using namespace std;

class Camera
{
private:
	glm::mat4 projection;
	glm::mat4 cameraPositionMatrix;

	glm::vec3 cameraPos;
	glm::vec3 worldUp;
	glm::vec3 Center;

	glm::vec3 F;
	glm::vec3 R;
	glm::vec3 U;

	glm::mat4 cameraOrientation;
	glm::mat4 viewMatrix;

public:
	Camera(float width, float height);
	glm::mat4 GetProjection();
	glm::mat4 GetViewMatrix();
	void SetPerspective(float width, float height);

};

