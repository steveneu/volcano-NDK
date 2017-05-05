precision mediump float;

uniform sampler2D colorMap;
//
//varying vec4 vFragColor; // <- shader compiler dosen't like this line
//smooth in vec2 vVaryingTexCoords; // <- shader compiler dosen't like this line
varying vec2 vVaryingTexCoords;

void main()
{
	gl_FragColor = texture2D(colorMap, vVaryingTexCoords);
	//vec4 vFragColor = texture2D(colorMap, vVaryingTexCoords.st);
	//gl_FragColor = vec4(0.63671875, 0.76953125, 0, 1.0);//0.22265625, 1.0);
}
