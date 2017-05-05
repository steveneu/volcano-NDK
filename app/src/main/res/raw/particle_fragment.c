
precision mediump float;  
varying mediump vec4 fscolor;
//uniform sampler2D colorMap;
//varying vec2 vVaryingTexCoords;

void main()
{
	//gl_FragColor = fscolor;
	gl_FragColor.r = fscolor.r;
	gl_FragColor.g = fscolor.g;
	gl_FragColor.b = fscolor.b;
	gl_FragColor.a = fscolor.a;

//	gl_FragColor = vcolor;
			//texture2D(colorMap, vVaryingTexCoords);
}        

