package com.neusoft.particle;

import java.nio.ByteBuffer;

import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.lang.Math;
import java.util.Random;
import java.nio.ShortBuffer;
import java.util.ArrayList;
import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

import com.neusoft.particle.R;

import android.content.Context;
import android.content.res.Resources;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.opengl.Matrix;
import android.os.SystemClock;
import android.util.DisplayMetrics;
import android.util.Log;
import android.view.View;


// particle system effects
//
// fountain with particles shooting up and gravity applied
// gravity using device accelerometer
// changing colors
// transparency over lifetime
// vary the frequency of particle generation
// variance applied to direction vector
// variance applied to brightness
// incorporate gravitational point source
// fluctuation about axis of travel 'world of tanks' style
//package com.android.gl2jni;


// class SimulationView extends View implements SensorEventListener {
public class ParticleRenderer implements GLSurfaceView.Renderer 
{
	Random rndGenerator;
    
    private Context context;
    private ByteBuffer texdata;
    private FloatBuffer vertices;
    private FloatBuffer pointcolor;
    private ShortBuffer indicies;
    private String vertex_program;
    private String fragment_program;

    public ParticleRenderer(Context ctxt) { }
    
    // handler for user touching screen
    public void touchdownHandler(int x, int y)
    {
	   	 //DisplayMetrics metrics = new DisplayMetrics();
	   	 //getWindowManager().getDefaultDisplay().getMetrics(metrics);
		ObjectJNI.jni_touchdownHandler();
    }
    
    public void receiveMatrix(float[] mO)
    {
		ObjectJNI.jni_receiveMatrix(mO);
    }
    
    private int loadShader(int type, String shaderCode)
    {
		return 0;
    }
    
    // read shader source code from raw project resources, compile them, attach to the program,
    // and link.  a program id is created and returned by this function
    // returns: the shader program ID
    public int makeProgramFromShaders(int vert, int frag)
    {
		return 0;
    }

	// called when surface is created or re-created
	// https://developer.android.com/reference/android/opengl/GLSurfaceView.Renderer.html#onSurfaceCreated(javax.microedition.khronos.opengles.GL10, javax.microedition.khronos.egl.EGLConfig)
	// put code to create resources that need to be created when the rendering starts, and that need
	// to be recreated when the EGL context is lost.
	// override from GLSurfaceView.Renderer
    public void onSurfaceCreated(GL10 unused, EGLConfig config) 
    {  	
    	// public static native void initialize(int width, int height, String strVertexSrc, String strFragmentSrc);
    	
    	//ObjectJNI.jni_initialize("vshader", "fshader");
		ObjectJNI.jni_initialize(vertex_program, fragment_program);
//    	mProgram_particles = makeProgramFromShaders(R.raw.particle_vertex, R.raw.particle_fragment);
//    	
//        // get handle to the vertex shader's vPosition attribute
//        vertex_attrib_idx = GLES20.glGetAttribLocation(mProgram_particles, "vPosition");
//        color_attrib_idx = GLES20.glGetAttribLocation(mProgram_particles, "vColor");
//        
//        mProgram_texmesh = makeProgramFromShaders(R.raw.texmesh_vertex, R.raw.texmesh_fragment);
//        
//        // Set the background frame color
//        GLES20.glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    }
    
    public void onDrawFrame(GL10 unused) 
    {
    	int invalue=2;
        float[] rm = new float[16];
        float eyex, eyey, eyez, centerx, centery, centerz, upx, upy, upz;
        eyex = eyey = eyez = centerx = centery = centerz = upx = upy = upz = 0;

        Matrix.setLookAtM(rm, 0, eyex, eyey, eyez, centerx, centery, centerz,
                upx, upy, upz);
    	ObjectJNI.jni_drawframe(invalue);
    }
    
    // called when the 'view' is resized.  this can happen when the device is rotated 90 degrees for example
    public void onSurfaceChanged(GL10 unused, int width, int height) 
    {
		ObjectJNI.jni_surfaceChanged(width, height);
    }
    
    public void setFragmentProgram(String inFrag) {
    	fragment_program = inFrag;
    }
    
    public void setVertexProgram(String inVert) {
    	vertex_program = inVert;
    }
}