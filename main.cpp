#include<glad/glad.h>
#include <GLFW/glfw3.h>

#include<glm/glm.hpp>
#include<glm/gtc/matrix_transform.hpp>
#include<glm/gtc/type_ptr.hpp>

#include<string>
#include<iostream>

#include"MyShaders.h"
#include"Camera.h"

#include"Particle.h"
#include"Fireworks.h"
#include"ParticleGravity.h"
#include"ParticleForceRegistry.h"
#include"BasicSpringForce.h"
#include"AnchoredSpringForce.h"
#include"ElasticBungeeForce.h"
#include"ParticleContact.h"
#include"ParticleContactResolver.h"
#include"ParticleWorld.h"

#include"RigidBody.h"
#include"MySkybox.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include"tiny_obj_loader.h"

#define STB_IMAGE_IMPLEMENTATION
#include"stb_image.h"


#include "DebugManager.h"


using namespace std;

//Init callbacks
void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
void KeyCallBack(GLFWwindow* window, int key, int scancode, int action, int mods);


ParticleWorld world;


int main(void)
{

    DebugManager debug;

    // width and height of the screen
    float width = 1080;
    float height = 720;
    GLFWwindow* window;

    /* Initialize the library */
    if (!glfwInit())
        return -1;

    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(width, height, "Jae Macuha", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    /* Make the window's context current */
    glfwMakeContextCurrent(window);
    gladLoadGL();

    glViewport(0, 0, width, height);

    //Init mouse and key callbacks
    glfwSetMouseButtonCallback(window, MouseButtonCallback);
    glfwSetKeyCallback(window, KeyCallBack);

    //Create Shaders
    MyShaders* shader = new MyShaders("Shaders/shader.vert", "Shaders/shader.frag");
    MyShaders* redShader = new MyShaders("Shaders/redShader.vert", "Shaders/redShader.frag");

    // Delta Time
    float lastTime = glfwGetTime();

    //create camera
    Camera camera(width,height);
    MySkybox* skybox = new MySkybox();
    MyShaders* shader_Skybox = new MyShaders("Shaders/skybox.vert", "Shaders/skybox.frag");


    //================ TEST ==========================
   

    //============ PARTICLE WORLD TEST ===============
    world.initialize();

    //============START OF WHILE LOOP==============

    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window))
    {

        /* Render here */
        glClear(GL_COLOR_BUFFER_BIT);

        // Delta Time
        //current time
        GLfloat currTime = glfwGetTime();
        float deltaTime = currTime - lastTime;

        //shader_Skybox->UseShader();
        //skybox->Draw(shader_Skybox->GetShaderProgram(), camera);

        //================ TEST PARTICLE WORLD ===================
        world.startFrame();
        world.runPhysics(deltaTime);
        world.DrawUpdate(shader, camera);



        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        /* Poll for and process events */
        glfwPollEvents();

        lastTime = currTime; // delta time
    }
    
    //Free Memory
    world.FreeMemory();
    

    glfwTerminate();
    return 0;

}


// current ballistic selected
BallisticType currentBallistic = Pistol;

enum Direction
{
    UpperRight,
    UpperLeft,
    LowerRight,
    LowerLeft
};

Direction direction;

//Callback for Mouse Inputs
void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
    {
        cout << "Fire" << endl;
        if (currentBallistic  == Pistol || currentBallistic == Artillery || currentBallistic == Fireball || currentBallistic == Laser)
        {
            if (direction == UpperRight)
            {
                world.particles[world.particleCounter]->FireBallistic(currentBallistic, world.rb->position + glm::vec3(2, 2, 0));
            }
            else if (direction == UpperLeft)
            {
                world.particles[world.particleCounter]->FireBallistic(currentBallistic, world.rb->position + glm::vec3(-2, 2, 0));
            }
            else if (direction == LowerRight)
            {
                world.particles[world.particleCounter]->FireBallistic(currentBallistic, world.rb->position + glm::vec3(2, -2, 0));
            }
            else if (direction == LowerLeft)
            {
                world.particles[world.particleCounter]->FireBallistic(currentBallistic, world.rb->position + glm::vec3(-2, -2, 0));
            }
            
            world.particleCounter++;
            if (world.particleCounter > (world.particleLimit - 1))
            {
                world.particleCounter = 1; //We use 1 because the first particle is a placeholder
            }

        }
        
    }

}

//Callback for Keyboard Inputs
void KeyCallBack(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    //registry->clear();

    if (key == GLFW_KEY_1 && action == GLFW_PRESS) {
        currentBallistic = Pistol;
    }
    else if (key == GLFW_KEY_2 && action == GLFW_PRESS) {
        currentBallistic = Artillery;
    }
    else if (key == GLFW_KEY_3 && action == GLFW_PRESS) {
        currentBallistic = Fireball;
    }
    else if (key == GLFW_KEY_4 && action == GLFW_PRESS) {
        currentBallistic = Laser;
    }
    else if (key == GLFW_KEY_5 && action == GLFW_PRESS) {
        currentBallistic = FireworkEnum;
    }
    else if (key == GLFW_KEY_6 && action == GLFW_PRESS) {
        currentBallistic = BasicSpring;
    }
    else if (key == GLFW_KEY_7 && action == GLFW_PRESS) {
        currentBallistic = AnchoredSpring;
    }
    else if (key == GLFW_KEY_8 && action == GLFW_PRESS) {
        currentBallistic = ElasticSpring;
    }

    if (key == GLFW_KEY_Q && action == GLFW_PRESS) {
        direction = UpperLeft;
    }
    else if(key == GLFW_KEY_W && action == GLFW_PRESS) {
        direction = UpperRight;
    }
    else if (key == GLFW_KEY_S && action == GLFW_PRESS) {
        direction = LowerRight;
    }
    else if (key == GLFW_KEY_A && action == GLFW_PRESS) {
        direction = LowerLeft;
    }

    if (key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
        world.rb->ResetBox();
    }

}
