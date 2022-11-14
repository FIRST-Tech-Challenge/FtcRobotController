package org.Team6200;

public class MathUtil {
    public static int orientationToDegrees(float orientation){
        return (int)Math.round(orientation*57.325);
    }
    public static float degreesToOrientation(int degrees){
        return (float)(((float)degrees)/57.325);
    }
}
