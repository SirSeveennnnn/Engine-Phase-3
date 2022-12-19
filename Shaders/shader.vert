#version 330 core //Version Number

//the position variable has attribute position 0
//Accesses the vertex position and assigns it to aPos
layout(location = 0) in vec3 aPos; 

//Creates a Transform variable
//We'll assign the Transformation matrix here
uniform mat4 transform;
uniform mat4 projection;
uniform mat4 view;

void main()
{
    //gl_Position is predefined
    //This is the final position of the matrix when drawn
    gl_Position =   projection * view*
                    transform * //Multiply the matrix with the position
                    vec4(aPos, 1.0); // Turns our 3x1 matrix into a 4x1

}