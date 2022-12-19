#include "Camera.h"

Camera::Camera(float width, float height)
{
	SetPerspective(width, height);
	cameraPos = glm::vec3(0, 0, 10.f);
	cameraPositionMatrix = glm::translate(glm::mat4(1.0f), cameraPos * -1.0f);
	worldUp = glm::vec3(0, 1.0f, 0);
	glm::vec3 Center = glm::vec3(0, 3.f, 0);
	glm::vec3 F = glm::vec3(Center - cameraPos);
	F = glm::normalize(F);
	R = glm::normalize(glm::cross(F, worldUp));
	U = glm::normalize(glm::cross(R, F));
	cameraOrientation = glm::mat4(glm::vec4(R, 0), glm::vec4(U, 0), glm::vec4((F * -1.0f), 0), glm::vec4(glm::vec3(0, 0, 0), 1));
	viewMatrix = cameraOrientation * cameraPositionMatrix;

}

glm::mat4 Camera::GetProjection()
{
	return this->projection;
}

glm::mat4 Camera::GetViewMatrix()
{
	return this->viewMatrix;
}

void Camera::SetPerspective(float width, float height)
{
	projection = glm::perspective(glm::radians(90.0f), (width / height), 0.f, 100.0f);
}
