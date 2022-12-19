#include "ParticleWorld.h"

void ParticleWorld::startFrame()
{

}

void ParticleWorld::runPhysics(float deltaTime)
{
	registry.UpdateForces(deltaTime);

	integrate(deltaTime);

	generateContacts();

	if (contacts.size() > 0)
	{
		contactResolver->maxIterations = contacts.size() * 3;
		contactResolver->ResolveContacts(contacts, deltaTime);
	}

}

void ParticleWorld::integrate(float deltaTime)
{
	for (int i = 0; i < particles.size(); i++)
	{
		particles[i]->Update(deltaTime);
	}

	for (int i = 0; i < boxParticles.size(); i++)
	{
		boxParticles[i]->Update(deltaTime);
	}

	rb->integrate(deltaTime);
}

float ParticleWorld::Distance(glm::vec3 a, glm::vec3 b)
{
	return glm::length(a - b);
}


void ParticleWorld::DrawUpdate(MyShaders* shader, Camera camera)
{
	shader->UseShader();
	for (int i = 0; i < particles.size(); i++)
	{
		particles[i]->Draw(shader->GetShaderProgram(), camera);
	}

	for (int i = 0; i < boxParticles.size(); i++)
	{
		boxParticles[i]->Draw(shader->GetShaderProgram(), camera);
	}

	rb->Draw(shader->GetShaderProgram(), camera);
}

void ParticleWorld::initialize()
{
	for (int i = 0; i < particleLimit; i++)
	{
		particles.push_back(new Particle(glm::vec3(-30, -30.f, -10.f)));
	}

	rb = new RigidBody();
	rb->mass = 1;
	rb->setInverseInertiaTensor();
	
}

void ParticleWorld::FreeMemory()
{
	for (int i = 0; i < particles.size(); i++)
	{
		particles[i]->Destroy();
	}
}

void ParticleWorld::addContacts(Particle* a, Particle* b)
{
	//addcontact
	ParticleContact* contact = new ParticleContact();
	glm::vec3 dir = a->GetVelocity();
	float length = glm::length(dir);
	dir.x = dir.x / length;
	dir.y = dir.y / length;
	dir.z = dir.z / length;

	glm::vec3 dir2 = a->GetPosition() - b->GetPosition();
	float length2 = glm::length(dir2);
	dir2.x = dir2.x / length2;
	dir2.y = dir2.y / length2;
	dir2.z = dir2.z / length2;

	glm::vec3 finalDir = -dir + dir2;
	float finalLength = glm::length(finalDir);
	finalDir.x = finalDir.x / finalLength;
	finalDir.y = finalDir.y / finalLength;
	finalDir.z = finalDir.z / finalLength;
	

	contact->contactNormal = finalDir;//glm::vec3(-0.91, 0, 0.39);

	contact->restitution = 1;
	contact->particle[0] = a;
	contact->particle[1] = b;

	contacts.push_back(contact);

}

void ParticleWorld::generateContacts()
{
	//Collision detection with box
	contacts.clear();

	//Detect Particle To Box Collision
	for (int i = 0; i < particles.size(); i++)
	{
		if (ParticleCubeCollision(particles[i], rb)) {

			cout << "Collision" << endl;
			//Apply force & torque
			glm::vec3 deltaVelocity = rb->velocity - particles[i]->GetVelocity();
			float totalMass = rb->mass + particles[i]->GetInverseMass();
			glm::vec3 impulsePerMass = deltaVelocity / totalMass;

			rb->velocity += -(impulsePerMass * rb->mass);
			rb->addTorque(particles[i]->GetPosition(), particles[i]->GetVelocity());


			//Reset Particle
			particles[i]->ResetParticle();

		}
	}

}

bool ParticleWorld::ParticleCubeCollision(Particle* particle, RigidBody* rb)
{
	float sphereXDistance = abs(particle->GetPosition().x - rb->position.x);
	float sphereYDistance = abs(particle->GetPosition().y - rb->position.y);
	float sphereZDistance = abs(particle->GetPosition().z - rb->position.z);

	if (sphereXDistance >= (rb->scale.x + 0.5)) { return false; }
	if (sphereYDistance >= (rb->scale.y + 0.5)) { return false; }
	if (sphereZDistance >= (rb->scale.z + 0.5)) { return false; }

	if (sphereXDistance < (rb->scale.x)) { return true; }
	if (sphereYDistance < (rb->scale.y)) { return true; }
	if (sphereZDistance < (rb->scale.z)) { return true; }

	float cornerDistance_sq = ((sphereXDistance - rb->scale.x) * (sphereXDistance - rb->scale.x)) +
		((sphereYDistance - rb->scale.y) * (sphereYDistance - rb->scale.y) +
			((sphereYDistance - rb->scale.z) * (sphereYDistance - rb->scale.z)));

	return (cornerDistance_sq < (0.5 * 0.5));
}

