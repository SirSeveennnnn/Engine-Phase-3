#include "MyShaders.h"

MyShaders::MyShaders(string vert, string frag)
{
    std::fstream VertStream(vert);
    std::stringstream VertStrStream;
    VertStrStream << VertStream.rdbuf();
    std::string VertStr = VertStrStream.str();

    const char* VertSrc = VertStr.c_str();

    std::fstream FragStream(frag);
    std::stringstream FragStrStream;
    FragStrStream << FragStream.rdbuf();
    std::string FragStr = FragStrStream.str();

    const char* FragSrc = FragStr.c_str();

    GLuint VertShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(VertShader, 1, &VertSrc, NULL);
    glCompileShader(VertShader);

    GLuint FragShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(FragShader, 1, &FragSrc, NULL);
    glCompileShader(FragShader);

    ShaderProgram = glCreateProgram();
    glAttachShader(ShaderProgram, VertShader);
    glAttachShader(ShaderProgram, FragShader);

    glLinkProgram(ShaderProgram);
}

GLuint MyShaders::GetShaderProgram()
{
    return this->ShaderProgram;
}

void MyShaders::UseShader()
{
    glUseProgram(ShaderProgram);
}
