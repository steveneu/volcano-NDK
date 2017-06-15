package com.neusoft.particle;

// Wrapper for native library
public class
ObjectJNI {
	static {
		System.loadLibrary("particle-jni");
	}

	public static native void jni_initialize(String strVertexSrc, String strFragmentSrc,
											 String strTextureVp, String strTextureFp);
	public static native void jni_drawframe(/*int invalue*/);
	public static native void jni_surfaceChanged(int inwidth, int inheight);
	public static native void jni_touchdownHandler();
	public static native void jni_receiveMatrix(float[] jfa);
	public static native void jni_pushTexture(int[] arr, int w, int h);

	//public static native void Java_com_neusoft_particle_jni_initialize1(String strVertexSrc, String strFragmentSrc);

/*
	JNIEXPORT void JNICALL Java_com_neusoft_particle_ObjectJNI_jni_1surfaceChanged
	    (JNIEnv *, jclass, jint inwidth, jint inheight)
	JNIEXPORT void JNICALL Java_com_neusoft_particle_ObjectJNI_jni_1touchdownHandler
	    (JNIEnv *, jclass, jint invalue)
	JNIEXPORT void JNICALL Java_com_neusoft_particle_ObjectJNI_jni_1receivematrix
		(JNIEnv *env, jclass, jfloatArray jfa)
	*/
}
