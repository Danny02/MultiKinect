#version 120
#pragma include includes/StdLib.vert

in vec3 in_position;    //<Position>
out vec3 normal;
out vec3 eye;

uniform mat4 view; //<MAT_V>
uniform mat4 mvp; //<MAT_MVP>

void main()
{
    vec3 p = in_position;
    normal = p;

    vec4 p2 = view * vec4(p,1);
    eye = -p2.xyz;

    gl_Position =  mvp * vec4(p, 1.);
}