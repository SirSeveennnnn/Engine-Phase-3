#include "MySkybox.h"

MySkybox::MySkybox()
{
    float skyboxVertices[]{
  -1.f, -1.f, 1.f, //0
  1.f, -1.f, 1.f,  //1
  1.f, -1.f, -1.f, //2
  -1.f, -1.f, -1.f,//3
  -1.f, 1.f, 1.f,  //4
  1.f, 1.f, 1.f,   //5
  1.f, 1.f, -1.f,  //6
  -1.f, 1.f, -1.f  //7
    };

    //Skybox Indices
    unsigned int skyboxIndices[]{
        1,2,6,
        6,5,1,

        0,4,7,
        7,3,0,

        4,5,6,
        6,7,4,

        0,3,2,
        2,1,0,

        0,1,5,
        5,4,0,

        3,7,6,
        6,2,3
    };

    std::string facesSkybox[]{
        "Textures/left.jpg",
        "Textures/right.jpg",
        "Textures/top.jpg",
        "Textures/down.jpg",
        "Textures/front.jpg",
        "Textures/back.jpg"

    };


    glGenVertexArrays(1, &skyboxVAO);
    glGenBuffers(1, &skyboxVBO);
    glGenBuffers(1, &skyboxEBO);

    glBindVertexArray(skyboxVAO);
    glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(skyboxVertices), &skyboxVertices, GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, skyboxEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GL_INT) * 36, &skyboxIndices, GL_DYNAMIC_DRAW);

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);


    glGenTextures(1, &skyBoxTex);
    glBindTexture(GL_TEXTURE_CUBE_MAP, skyBoxTex);

    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    for (unsigned int i = 0; i < 6; i++)
    {
        int w, h, skyCChannel;
        stbi_set_flip_vertically_on_load(false);

        unsigned char* data = stbi_load(facesSkybox[i].c_str(), &w, &h, &skyCChannel, 0);

        if (data)
        {
            glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
            stbi_image_free(data);

        }

    }
    stbi_set_flip_vertically_on_load(true);
}

void MySkybox::Draw(GLuint ShaderProgram, Camera camera)
{
    glDepthMask(GL_FALSE);
    glDepthFunc(GL_LEQUAL);

    glm::mat4 skyView = glm::mat4(1.0f);
    skyView = glm::mat4(glm::mat3(camera.GetViewMatrix()));

    
	skyView = glm::mat4(glm::mat3(camera.GetViewMatrix()));

	unsigned int skyboxViewLoc = glGetUniformLocation(ShaderProgram, "view");
	glUniformMatrix4fv(skyboxViewLoc, 1, GL_FALSE, glm::value_ptr(skyView));

	unsigned int skyboxProjLoc = glGetUniformLocation(ShaderProgram, "projection");
	glUniformMatrix4fv(skyboxProjLoc, 1, GL_FALSE, glm::value_ptr(camera.GetProjection()));
    
    glBindVertexArray(skyboxVAO);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_CUBE_MAP, skyBoxTex);
    glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);

    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);
}

