#include "Line.h"

void Line::Draw(glm::vec3 a, glm::vec3 b)
{
	glBegin(GL_LINES);
	{
		glVertex3f(a.x, a.y, a.z);
		glVertex3f(b.x, b.y, b.z);
	}
	glEnd();
}
