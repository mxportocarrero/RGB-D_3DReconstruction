#version 430

in vec3 FragmentColor;

out vec4 FragColor;

void main(void)
{
    FragColor = vec4(FragmentColor,1.0f);
    //FragColor = vec4(1.0,1.0,0.5,1.0f);
}
