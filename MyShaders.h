#pragma once

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

#include<glad/glad.h>
#include <GLFW/glfw3.h>

#include<glm/glm.hpp>
#include<glm/gtc/matrix_transform.hpp>
#include<glm/gtc/type_ptr.hpp>

using namespace std;


class MyShaders
{
public:
	MyShaders(string vert, string frag);
	GLuint GetShaderProgram();
	void UseShader();
private:
	GLuint ShaderProgram;

};

