
precision mediump float;  
varying mediump vec4 fscolor;


void main()
{
	gl_FragColor.r = fscolor.r;
	gl_FragColor.g = fscolor.g;
	gl_FragColor.b = fscolor.b;
	gl_FragColor.a = fscolor.a;
}

