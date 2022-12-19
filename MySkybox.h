#pragma once
#include<glad/glad.h>
#include <GLFW/glfw3.h>

#include<glm/glm.hpp>
#include<glm/gtc/matrix_transform.hpp>
#include<glm/gtc/type_ptr.hpp>

#include<string>
#include<vector>
#include<iostream>

#include"tiny_obj_loader.h"

#include"stb_image.h"

#include"Camera.h"


#include<unordered_map>

using namespace std;

using namespace std;

class MySkybox
{
public:
	MySkybox();
	void Draw(GLuint ShaderProgram, Camera camera);

private:
	GLuint skyboxVAO, skyboxVBO, skyboxEBO;
	unsigned int skyBoxTex;
};

