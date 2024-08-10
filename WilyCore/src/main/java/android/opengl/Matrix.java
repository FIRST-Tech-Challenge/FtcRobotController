//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package android.opengl;

public class Matrix {
    /** @deprecated */
    @Deprecated
    public Matrix() {
        throw new RuntimeException("Stub!");
    }

    public static native void multiplyMM(float[] var0, int var1, float[] var2, int var3, float[] var4, int var5);

    public static native void multiplyMV(float[] var0, int var1, float[] var2, int var3, float[] var4, int var5);

    public static void transposeM(float[] mTrans, int mTransOffset, float[] m, int mOffset) {
        throw new RuntimeException("Stub!");
    }

    public static boolean invertM(float[] mInv, int mInvOffset, float[] m, int mOffset) {
        throw new RuntimeException("Stub!");
    }

    public static void orthoM(float[] m, int mOffset, float left, float right, float bottom, float top, float near, float far) {
        throw new RuntimeException("Stub!");
    }

    public static void frustumM(float[] m, int offset, float left, float right, float bottom, float top, float near, float far) {
        throw new RuntimeException("Stub!");
    }

    public static void perspectiveM(float[] m, int offset, float fovy, float aspect, float zNear, float zFar) {
        throw new RuntimeException("Stub!");
    }

    public static float length(float x, float y, float z) {
        throw new RuntimeException("Stub!");
    }

    public static void setIdentityM(float[] sm, int smOffset) {
        // ### throw new RuntimeException("Stub!");
    }

    public static void scaleM(float[] sm, int smOffset, float[] m, int mOffset, float x, float y, float z) {
        throw new RuntimeException("Stub!");
    }

    public static void scaleM(float[] m, int mOffset, float x, float y, float z) {
        throw new RuntimeException("Stub!");
    }

    public static void translateM(float[] tm, int tmOffset, float[] m, int mOffset, float x, float y, float z) {
        throw new RuntimeException("Stub!");
    }

    public static void translateM(float[] m, int mOffset, float x, float y, float z) {
        throw new RuntimeException("Stub!");
    }

    public static void rotateM(float[] rm, int rmOffset, float[] m, int mOffset, float a, float x, float y, float z) {
        throw new RuntimeException("Stub!");
    }

    public static void rotateM(float[] m, int mOffset, float a, float x, float y, float z) {
        throw new RuntimeException("Stub!");
    }

    public static void setRotateM(float[] rm, int rmOffset, float a, float x, float y, float z) {
        throw new RuntimeException("Stub!");
    }

    public static void setRotateEulerM(float[] rm, int rmOffset, float x, float y, float z) {
        throw new RuntimeException("Stub!");
    }

    public static void setLookAtM(float[] rm, int rmOffset, float eyeX, float eyeY, float eyeZ, float centerX, float centerY, float centerZ, float upX, float upY, float upZ) {
        throw new RuntimeException("Stub!");
    }
}
