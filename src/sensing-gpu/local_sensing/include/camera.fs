#version 330 core
out vec4 FragColor;
in vec3 ourColor;
flat in vec2 coverage;

void main()
{
    if (any(greaterThan(abs(2.0 * gl_PointCoord - 1.0), coverage)))
        discard;
    FragColor = vec4(ourColor, 1.0);
}
