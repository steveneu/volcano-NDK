#version 100

uniform sampler2D colorMap;

in vec4 vVaryingTexCoords;
out vec4 vFragColor;

precision mediump float;
//varying mediump vec4 fscolor;

void main()
{
    //gl_FragColor = fscolor;
    vFragColor = texture(colorMap, vVaryingTexCoords.st);
}
