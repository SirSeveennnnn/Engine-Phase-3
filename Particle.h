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


#include"Camera.h"


#include<unordered_map>

using namespace std;

enum BallisticType {
    Pistol,
    Artillery,
    Fireball,
    Laser,
    FireworkEnum,
    BasicSpring,
    AnchoredSpring,
    ElasticSpring
};

class Particle
{
public:
    Particle(glm::vec3 point);
    ~Particle();
    void SetSpawnPoint(glm::vec3 point);
    void LoadModel();
    void LoadVaoVboEbo();
    void LoadTransform();
    void Draw(GLuint bunnyShaderProgram, Camera camera);
    void SetPosition(glm::vec3 pos);
    void Destroy();

    virtual void Update(float deltaTime);
    virtual void integrate(float deltaTime);
    void AddForce(glm::vec3 force);
    void UpdatePositionVector();

    void SetVelocity(glm::vec3 vel);
    void SetAcceleration(glm::vec3 accel);
    void ResetResultingAcceleration();
    void SetInverseMass(float mass);
    void SetDamping(float damp);
    void SetLifeTime(float time);

    glm::vec3 GetPosition();
    glm::vec3 GetVelocity();
    float GetInverseMass();


    void ResetParticle();

    void FirePistol(glm::vec3 target, float speed);
    void FireArtillery();
    void FireFireball();
    void FireLaser(glm::vec3 target, float speed);
    void FireBallistic(BallisticType type, glm::vec3 target);

    void FireSpring();

    

    //if shot is used
    bool isUsed = false;
    bool hasCollided = false;

protected:
  
    //Model Variables
    std::string bunnyPath = "Models/sphere.obj";
    std::vector<tinyobj::shape_t> sphereShapes;
    std::vector<tinyobj::material_t> bunnyMaterial;
    std::string warning, error;

    tinyobj::attrib_t bunnyAttributes;

    bool bunnySuccess = tinyobj::LoadObj(&bunnyAttributes, &sphereShapes, &bunnyMaterial, &warning, &error, bunnyPath.c_str());

    std::vector<GLuint> sphereMesh_indices;

    GLuint bVAO, bVBO, bEBO;

    //Matrices and Vectors
    
    //Identity matrix reference
    glm::mat4 identity_matrix4 = glm::mat4(1.0f);
    glm::vec3 spawnPoint = glm::vec3(0, -30.f, 0.f);

    //Transformation Matrix of the Particle
    glm::mat4 Transform;

    //Position of the Particle
    glm::vec3 position;

    //Velocity of the Particle
    glm::vec3 velocity;

    //Acceleration of the Particle
    glm::vec3 acceleration;
    glm::vec3 resultingAcceleration = glm::vec3(0, 0, 0);


    //Damping Variable
    float dampingMultiplier;

    //InverseMass
    float inverseMass;

    //Current time the particle is active
    float currentLifeTime = 0;

    //Particle's lifespan
    float lifeTime = 5.f;

    
    
};

