#include "RigidBody.h"

RigidBody::RigidBody()
{
    SetSpawnPoint(spawnPoint);
    LoadModel();
    LoadVaoVboEbo();
    LoadTransform();
}

void RigidBody::SetSpawnPoint(glm::vec3 point)
{
    this->spawnPoint = point;
}

void RigidBody::CalculateDerivedData()
{
    //position
    transform = glm::translate(identity_matrix4, position); // x,y,z
    //rotation
    glm::vec3 normalizedRotation;
    float rotationMagnitude = glm::length(orientation);
    if (rotationMagnitude != 0)
    {
        normalizedRotation.x = orientation.x / rotationMagnitude;
        normalizedRotation.y = orientation.y / rotationMagnitude;
        normalizedRotation.z = orientation.z / rotationMagnitude;

        transform = glm::rotate(transform, glm::radians(rotationMagnitude), normalizedRotation);
    }
    
    //transform *= glm::toMat4(quaternion);

    //scale
    transform = glm::scale(transform, scale); // x,y,z

    

}

void RigidBody::Draw(GLuint shaderProgram, Camera camera)
{
    unsigned int projLoc = glGetUniformLocation(shaderProgram, "projection");
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(camera.GetProjection()));

    unsigned int viewLoc = glGetUniformLocation(shaderProgram, "view");
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(camera.GetViewMatrix()));

    unsigned int bunnyTransformLoc = glGetUniformLocation(shaderProgram, "transform");
    glUniformMatrix4fv(bunnyTransformLoc, 1, GL_FALSE, glm::value_ptr(transform));

    glUseProgram(shaderProgram);

    glBindVertexArray(bVAO);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glDrawElements(GL_TRIANGLES, sphereMesh_indices.size(), GL_UNSIGNED_INT, 0);
}

void RigidBody::LoadModel()
{
    for (int i = 0; i < sphereShapes[0].mesh.indices.size(); i++)
    {
        sphereMesh_indices.push_back(sphereShapes[0].mesh.indices[i].vertex_index);
    }
}

void RigidBody::LoadVaoVboEbo()
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

void RigidBody::LoadTransform()
{

    //position
    transform = glm::translate(identity_matrix4, spawnPoint); // x,y,z
    position = spawnPoint;
    //rotation
    
    glm::vec3 normalizedRotation;
    float rotationMagnitude = glm::length(orientation);
   
    if (rotationMagnitude != 0)
    {
        normalizedRotation.x = orientation.x / rotationMagnitude;
        normalizedRotation.y = orientation.y / rotationMagnitude;
        normalizedRotation.z = orientation.z / rotationMagnitude;

        transform = glm::rotate(transform, glm::radians(rotationMagnitude), normalizedRotation);
    }

    //scale
    transform = glm::scale(transform, scale); // x,y,z

}

void RigidBody::addForce(glm::vec3 force)
{
    forceAccum += force * mass;
}

void RigidBody::clearAccumulators() 
{
    forceAccum = glm::vec3(0, 0, 0);
    torqueAccum = glm::vec3(0, 0, 0);
}

void RigidBody::CalculateEulerAngle()
{
    rotation = glm::degrees(glm::eulerAngles(quaternion));

}

void RigidBody::setInverseInertiaTensor()
{
    float tensorX = (1.0f / 12.0f) * mass * (scale.y * scale.y + scale.z * scale.z);
    float tensorY = (1.0f / 12.0f) * mass * (scale.x * scale.x + scale.z * scale.z);
    float tensorZ = (1.0f / 12.0f) * mass * (scale.x * scale.x + scale.y * scale.y);

    inverseInertiaTensor[0][0] = tensorX;
    inverseInertiaTensor[1][1] = tensorY;
    inverseInertiaTensor[2][2] = tensorZ;
    
}

void RigidBody::Update(float deltaTime)
{
}

void RigidBody::integrate(float deltaTime)
{
    forceAccum += acceleration * mass; 

   //Add scaled vectors
    velocity += forceAccum * deltaTime;

    //update rotation
    glm::vec3 angularVelocity = inverseInertiaTensor * torqueAccum;
    

    rotation += angularVelocity * deltaTime; //try delta time
    //cout << "rotation X: " << rotation.x << " Y: " << rotation.y << " Z: " << rotation.z << endl;

    //add drag to the velocity
    velocity *= linearDamp;
    rotation *= angularDamp;

    //update position and rotation
    position += velocity * deltaTime + 0.5f * forceAccum * deltaTime * deltaTime;
    orientation += rotation * deltaTime;

    quaternion += glm::quat(rotation);

    CalculateDerivedData();

    //clear forces
    clearAccumulators();
}

void RigidBody::ResetBox()
{
    position = spawnPoint;
    rotation = glm::vec3(0, 0, 0);
    orientation = glm::vec3(0, 45, 0);
    velocity = glm::vec3(0, 0, 0);
}


void RigidBody::addTorque(glm::vec3 point, glm::vec3 force)
{
    //Convert to local space
    glm::vec3 localPoint = point - position;

    // negative sign because box is on the receiving end
    torqueAccum += -glm::cross(force, localPoint);

}


