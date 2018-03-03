#version 330

in vec3 FragmentColor;

out vec4 FragColor;

void main(void)
{
    FragColor = vec4(FragmentColor,1.0f);
}
