#version 100

uniform sampler2D texture_unit;

varying vec2 vVaryingTexCoords;

void main()
{
    gl_FragColor = texture2D(texture_unit, vVaryingTexCoords);
}
