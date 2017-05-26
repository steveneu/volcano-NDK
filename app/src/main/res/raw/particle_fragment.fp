#version 100

precision mediump float;  
varying mediump vec4 fscolor;

void main()
{
    gl_FragColor = fscolor;
}

