package com.neusoft.particle;
//package com.example.android.apis.graphics;

import android.app.Activity;
import android.content.Context;
import android.content.pm.ActivityInfo;
import android.content.res.Configuration;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.opengl.GLSurfaceView;
import android.os.Bundle;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import android.util.Log;
import android.view.MotionEvent;


public class Particle extends Activity {
  
    //private GLSurfaceView mGLView;
    private SensorManager mSensorManager;
    HelloOpenGLES20SurfaceView glsv;
  
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        
        // lock device orientation 
        switch (getResources().getConfiguration().orientation)
        {
            case Configuration.ORIENTATION_PORTRAIT:
                if(android.os.Build.VERSION.SDK_INT < android.os.Build.VERSION_CODES.FROYO)
    			{
                    setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);
                } 
    			else 
    			{
                    int rotation = getWindowManager().getDefaultDisplay().getRotation();
    				
    				if (rotation == android.view.Surface.ROTATION_90|| rotation == android.view.Surface.ROTATION_180)
                        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_REVERSE_PORTRAIT);
    				else 
    					 setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);
    			}   
            break;

            case Configuration.ORIENTATION_LANDSCAPE:
                if(android.os.Build.VERSION.SDK_INT < android.os.Build.VERSION_CODES.FROYO)
    			{
                    setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
                } 
    			else 
    			{
                    int rotation = getWindowManager().getDefaultDisplay().getRotation();
                    
                    if(rotation == android.view.Surface.ROTATION_0 || rotation == android.view.Surface.ROTATION_90)
                        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
    				else
                        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_REVERSE_LANDSCAPE);
                }
            break;
        }
        
        // Get an instance of the SensorManager
        mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        
        // Create a GLSurfaceView instance and set it
        // as the ContentView for this Activity
        glsv = new HelloOpenGLES20SurfaceView(this);
        setContentView(glsv);
    };
    
    @Override
    protected void onPause() {
        super.onPause();
        // The following call pauses the rendering thread.
        // If your OpenGL application is memory intensive,
        // you should consider de-allocating objects that
        // consume significant memory here.
        
        glsv.stopSimulation();
        glsv.onPause();
    }
    
    @Override
    protected void onResume() {
        super.onResume();
        // The following call resumes a paused rendering thread.
        // If you de-allocated graphic objects for onPause()
        // this is a good place to re-allocate them.
        glsv.startSimulation();
        
        glsv.onResume();
    }
    
    class HelloOpenGLES20SurfaceView extends GLSurfaceView implements SensorEventListener {

		private Sensor sensorGravity;
		private Sensor sensorMagneticField;
		private Sensor sensorRotationVector;
		private ParticleRenderer renderer;

		private float magneticfield[] = new float[3];
		private Boolean capturedmagneticfield;

		public HelloOpenGLES20SurfaceView(Context context) {
			super(context);

//            sensorGravity = mSensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);
			//          sensorMagneticField = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
			sensorRotationVector = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);

			// Create an OpenGL ES 2.0 context.
			setEGLContextClientVersion(2);
			// Set the Renderer for drawing on the GLSurfaceView
			renderer = new ParticleRenderer(context);

			String vertSrc = readRawTextFile(context, R.raw.particle_vertex);
			String fragSrc = readRawTextFile(context, R.raw.particle_fragment);

			String vertTextureSrc = readRawTextFile(context, R.raw.texmesh_vertex);
			String fragTextureSrc = readRawTextFile(context, R.raw.texmesh_fragment);

			renderer.setVertexProgram(vertSrc);
			renderer.setFragmentProgram(fragSrc);
			renderer.setVertexTextureProgram(vertTextureSrc);
			renderer.setFragmentTextureProgram(fragTextureSrc);

			setRenderer(renderer);
			capturedmagneticfield = false;

			// read texture from resources
			int[] pixels;
			BitmapFactory.Options opts = new BitmapFactory.Options();
			opts.inDensity = 0;
			opts.inTargetDensity = 0;
			opts.inPremultiplied = false;
			opts.inScaled = false;
			opts.inSampleSize = 1;
			Bitmap bitmap = BitmapFactory.decodeResource(getResources(), R.raw.volcano_ground, opts); // volcano_ground, quilt_test
//			int rowbytes = bitmap.getRowBytes(); //Return the number of bytes between rows in the bitmap's pixels
//			boolean hasalpha = bitmap.hasAlpha();
			int bytecount = bitmap.getByteCount(); // Returns the minimum number of bytes that can be used to store this bitmap's pixels.
			int w = bitmap.getWidth(); // width in bytes? pixels x bpc
			int h = bitmap.getHeight();

			pixels = new int[bytecount/4]; // 4 bytes per channel if alpha present
			int stride = w;
			int rowPixels = w;//w;
			int rowstoread = h;//h;
			bitmap.getPixels(pixels, // The array to receive the bitmap's colors
					0, // offset- The first index to write into pixels[]
					stride, // stride- The number of entries in pixels[] to skip between rows (must be >= bitmap's width). Can be negative.
					0, // The x coordinate of the first pixel to read from the bitmap
					0, // The y coordinate of the first pixel to read from the bitmap
					rowPixels, // The number of pixels to read from each row
					rowstoread); // The number of rows to read

			// send textures to native side
			renderer.pokeBitmap(pixels, w, h);
		}

		public String readRawTextFile(Context ctx, int resId)
		{
			InputStream inputStream = ctx.getResources().openRawResource(resId);

			InputStreamReader inputreader = new InputStreamReader(inputStream);
			BufferedReader buffreader = new BufferedReader(inputreader);
			String line;
			StringBuilder text = new StringBuilder();

			try {
				while (( line = buffreader.readLine()) != null) {
					text.append(line);
					text.append('\n');
				}
			} catch (IOException e) {
				return null;
			}
			return text.toString();
		}

        public void startSimulation() {
            /*
             * It is not necessary to get accelerometer events at a very high
             * rate, by using a slower rate (SENSOR_DELAY_UI), we get an
             * automatic low-pass filter, which "extracts" the gravity component
             * of the acceleration. As an added benefit, we use less power and
             * CPU resources.
             */
        	
        	/*
        	 * int	SENSOR_DELAY_FASTEST	get sensor data as fast as possible
int	SENSOR_DELAY_GAME	rate suitable for games
int	SENSOR_DELAY_NORMAL	rate (default) suitable for screen orientation changes
int	SENSOR_DELAY_UI	rate suitable for the user interface
        	 * */
        	
            mSensorManager.registerListener(this, sensorRotationVector, SensorManager.SENSOR_DELAY_GAME);
        }

        public void stopSimulation() {
            mSensorManager.unregisterListener( this);
        }
        
    	public void onAccuracyChanged(Sensor sensor, int accuracy) {}
        
        public void onSensorChanged(SensorEvent event)
        {
        	if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR)
        	{
        		final float[] rotationMatrix  = new float [16];
                // convert the rotation-vector to a 4x4 matrix. the matrix
                // is interpreted by Open GL as the inverse of the
                // rotation-vector, which is what we want.
                SensorManager.getRotationMatrixFromVector(
                        rotationMatrix , event.values);
                
        		// send orientation matrix to renderer
            	queueEvent(new Runnable()
            	{
            		
            		// this method will be called on the rendering thread
            		public void run() {
            			renderer.receiveMatrix(rotationMatrix);
            		}
            	});
        		
        	}
        	else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD)
        	{
                magneticfield[0]=event.values[0];
                magneticfield[1]=event.values[1];
                magneticfield[2]=event.values[2];
                //Log.d("Particle", "Mag field 0:" + event.values[0] + " 1:"+event.values[1]+ " 2:"+ event.values[2]);
        		capturedmagneticfield=true;
        	}
        	
        	else if (event.sensor.getType() == Sensor.TYPE_GRAVITY && capturedmagneticfield == true)
        	{
                float gravity[] = new float[3];
                
                gravity[0]=event.values[0];
                gravity[1]=event.values[1];
                gravity[2]=event.values[2];
        		
                final float R[]=new float[16];
                boolean result=false;
                //public static boolean getRotationMatrix (float[] R, float[] I, float[] gravity, float[] geomagnetic)
            	result = SensorManager.getRotationMatrix(R, null, gravity, magneticfield );
            	if (result==false)
            	{
            		Log.d("Particle", "failed to get rotation matrix for orientation");
            	}
            	else
            	{
            		// send orientation matrix to renderer
                	queueEvent(new Runnable()
                	{
                		
                		// this method will be called on the rendering thread
                		public void run() {
                			renderer.receiveMatrix(R);
                		}
                	});
            		// dump rot matrix to log
            		Log.d("Particle", " ");
            		Log.d("Particle", "Orientation Matrix: ");
            		Log.d("Particle", "     " + R[0] + " " + R[1] + " " + R[2]);
            		Log.d("Particle", "     " + R[3] + " " + R[4] + " " + R[5]);
            		Log.d("Particle", "     " + R[6] + " " + R[7] + " " + R[8]);
            	}
            	
            	capturedmagneticfield = false;
        	}
        	else
        	{
        		return;
        	}
       	
        	//Log.d("Particle", "gravity[0]: x: " + -event.values[0] + " y:"+ -event.values[1] + " z:"+ -event.values[2]);
        }
        
        @Override
        public boolean onTouchEvent(final MotionEvent e) {
            // MotionEvent reports input details from the touch screen
            // and other input controls. In this case, you are only
            // interested in events where the touch position changed.

            //float x = e.getX();
            //float y = e.getY();

            switch (e.getAction()) 
            {
                case MotionEvent.ACTION_DOWN:
                	               	
                	queueEvent(new Runnable()
                	{
                		// this method will be called on the rendering thread
                		public void run() {
                			renderer.touchdownHandler((int)e.getX(), (int)e.getY());
                		}
                	});
					break;
            }
            return super.onTouchEvent(e);
        }
    }
}

