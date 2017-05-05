// This matrix member variable provides a hook to manipulate
// the coordinates of the objects that use this vertex shader

uniform mat4 uMVPMatrix;   

//attribute vec2 vTexCoords;
attribute vec4 vPosition;
attribute mediump vec4 vColor;

varying mediump vec4 fscolor;

//varying vec2 vVaryingTexCoords;

void main()
{ 
	//vTexCoords;
	//vVaryingTexCoords = vTexCoords;
	fscolor = vColor;

	// the matrix must be included as a modifier of gl_Position
	gl_Position = uMVPMatrix * vPosition; 
	gl_PointSize = 3.0;
}  

