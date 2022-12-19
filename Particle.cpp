#include"Particle.h"

//Constructor loads eveything
Particle::Particle(glm::vec3 point)
{
    SetSpawnPoint(point);
    LoadModel();
    LoadVaoVboEbo();
    LoadTransform();
}

Particle::~Particle()
{
}

void Particle::SetSpawnPoint(glm::vec3 point)
{
    this->spawnPoint = point;
}

//Load mesh
void Particle::LoadModel()
{
    for (int i = 0; i < sphereShapes[0].mesh.indices.size(); i++)
    {
        sphereMesh_indices.push_back(sphereShapes[0].mesh.indices[i].vertex_index);
    }
}

//Load Buffers
void Particle::LoadVaoVboEbo()
{
    glGenVertexArrays(1, &bVAO);
    glGenBuffers(1, &bVBO);
    glGenBuffers(1, &bEBO);

    glBindVertexArray(bVAO);
    glBindBuffer(GL_ARRAY_BUFFER, bVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GL_FLOAT) * bunnyAttributes.vertices.size(), bunnyAttributes.vertices.data(), GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * sphereMesh_indices.size(), sphereMesh_indices.data(), GL_DYNAMIC_DRAW);

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

//Set Particle to the starting position
void Particle::LoadTransform()
{
    Transform = glm::translate(identity_matrix4, spawnPoint); // x,y,z
    //scale
    Transform = glm::scale(Transform, glm::vec3(0.02f, 0.02f, 0.02f)); // x,y,z
    //rotation
    Transform = glm::rotate(Transform, glm::radians(0.0f), glm::vec3(1, 0, 0)); //theta, x,y,z
    
    this->SetPosition(spawnPoint);

}

//draw using shaders
void Particle::Draw(GLuint shaderProgram, Camera camera)
{

    unsigned int projLoc = glGetUniformLocation(shaderProgram, "projection");
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(camera.GetProjection()));

    unsigned int viewLoc = glGetUniformLocation(shaderProgram, "view");
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(camera.GetViewMatrix()));

    unsigned int bunnyTransformLoc = glGetUniformLocation(shaderProgram, "transform");
    glUniformMatrix4fv(bunnyTransformLoc, 1, GL_FALSE, glm::value_ptr(Transform));

    glUseProgram(shaderProgram);

    glBindVertexArray(bVAO);
    glDrawElements(GL_TRIANGLES, sphereMesh_indices.size(), GL_UNSIGNED_INT, 0);
}

//Sets the position of the particle using a vec3 to get the transformation matrix
void Particle::SetPosition(glm::vec3 pos)
{
    Transform = glm::translate(identity_matrix4, pos);
    Transform = glm::scale(Transform, glm::vec3(0.02f, 0.02f, 0.02f)); // x,y,z

    UpdatePositionVector();
}

//Destroys the model's buffers
void Particle::Destroy()
{
    glDeleteVertexArrays(1, &bVAO);
    glDeleteBuffers(1, &bVBO);
    glDeleteBuffers(1, &bEBO);
}

void Particle::Update(float deltaTime)
{
    //Check if particle is being used
    if (isUsed)
    {
        //Update the particle life time
        currentLifeTime += deltaTime;

        //Check if particle should reset
        if (currentLifeTime >= lifeTime)
        {
            //Reset
            ResetParticle();
            
        }

        integrate(deltaTime);
    }

}

//Physics
void Particle::integrate(float deltaTime)
{
    
    resultingAcceleration += acceleration * inverseMass; //(1 / inverseMass); //inverse mass is 1 / mass meaning 1/(1/mass)

	//update position
	position += velocity * deltaTime + 0.5f * resultingAcceleration * deltaTime * deltaTime;
	SetPosition(position);

    velocity += resultingAcceleration * deltaTime; 

	//add drag to the velocity
	velocity *= dampingMultiplier;

    //clear forces
    resultingAcceleration = glm::vec3(0.f, 0.f, 0.f);

}

void Particle::AddForce(glm::vec3 force) {
    resultingAcceleration += force * inverseMass;
}

//Gets the position of the particle using the transformation matrix
void Particle::UpdatePositionVector()
{
    position.x = Transform[3][0];
    position.y = Transform[3][1];
    position.z = Transform[3][2];

}

// Sets the velocity of the particle
void Particle::SetVelocity(glm::vec3 vel)
{
    this->velocity = vel;
}

// Sets the acceleration of the particle
void Particle::SetAcceleration(glm::vec3 accel)
{
    this->acceleration = accel;
   
}

void Particle::ResetResultingAcceleration()
{
    this->resultingAcceleration = glm::vec3(0, 0, 0);
}

//Sets the mass of the particle
void Particle::SetInverseMass(float mass)
{
    this->inverseMass = mass;
}

//Sets the damping of the particle
void Particle::SetDamping(float damp)
{
    this->dampingMultiplier = damp;
}

//Sets the Life time of the particle
void Particle::SetLifeTime(float time)
{
    this->lifeTime = time;
}

glm::vec3 Particle::GetPosition()
{
    return position;
}

glm::vec3 Particle::GetVelocity()
{
    return velocity;
}

float Particle::GetInverseMass()
{
    return this->inverseMass;
}

//Reset the particle
void Particle::ResetParticle()
{
    SetPosition(spawnPoint);
    SetVelocity(glm::vec3(0, 0, 0));
    SetAcceleration(glm::vec3(0, 0, 0));
    SetInverseMass(1);
    SetDamping(0.f);
    ResetResultingAcceleration();
    currentLifeTime = 0;
    isUsed = false;
    hasCollided = false;
}

//Fires Pistol Ballistic
void Particle::FirePistol(glm::vec3 target, float speed)
{
    if (isUsed)
    {
        return;
    }
    isUsed = true;
    hasCollided = false;
    currentLifeTime = 0;

    //direction
    glm::vec3 dir = target - this->position;

    //normalize dir
    float magnitude = glm::length(dir);
    dir.x = dir.x / magnitude;
    dir.y = dir.y / magnitude;
    dir.z = dir.z / magnitude;


    SetVelocity(dir * speed);
    SetAcceleration(glm::vec3(0, -1.f, 0));
    ResetResultingAcceleration();
    SetInverseMass(2);
    SetDamping(0.999f);
    SetLifeTime(5.f);
}

void Particle::FireArtillery()
{
    if (isUsed)
    {
        return;
    }
    isUsed = true;
    hasCollided = false;
    currentLifeTime = 0;

    SetVelocity(glm::vec3(70, 150.f, -35.f));
    SetAcceleration(glm::vec3(0, -10.f, 0));
    ResetResultingAcceleration();
    SetInverseMass(20);
    SetDamping(0.999f);
    SetLifeTime(1.5f);
}

void Particle::FireFireball()
{
    if (isUsed)
    {
        return;
    }
    isUsed = true;
    hasCollided = false;
    currentLifeTime = 0;

    SetVelocity(glm::vec3(30, 0.f, -15.f));
    SetAcceleration(glm::vec3(0, 1.f, 0));
    ResetResultingAcceleration();
    SetInverseMass(1);
    SetDamping(0.999f);
    SetLifeTime(7.f);
}

void Particle::FireLaser(glm::vec3 target, float speed)
{
    if (isUsed)
    {
        return;
    }
    isUsed = true;
    hasCollided = false;
    currentLifeTime = 0;

    //direction
    glm::vec3 dir = target - this->position;

    //normalize dir
    float magnitude = glm::length(dir);
    dir.x = dir.x / magnitude;
    dir.y = dir.y / magnitude;
    dir.z = dir.z / magnitude;


    SetVelocity(dir * speed);
    SetAcceleration(glm::vec3(0, 0, 0));
    ResetResultingAcceleration();
    SetInverseMass(0.1f);
    SetDamping(0.999f);
    SetLifeTime(3.f);
}

void Particle::FireBallistic(BallisticType type, glm::vec3 target)
{
    switch (type)
    {
    case Pistol:
        FirePistol(target, 100);
        break;
    case Artillery:
        FireArtillery();
        break;
    case Fireball:
        FireFireball();
        break;
    case Laser:
        FireLaser(target, 200);
        break;
    }
}

void Particle::FireSpring()
{
    if (isUsed)
    {
        return;
    }
    isUsed = true;
    currentLifeTime = 0;

    SetVelocity(glm::vec3(0.f, 0.f, -50.f));
    SetAcceleration(glm::vec3(0, 0, 0));
    ResetResultingAcceleration();
    SetInverseMass(1);
    SetDamping(0.999f);
    SetLifeTime(5.f);
}



