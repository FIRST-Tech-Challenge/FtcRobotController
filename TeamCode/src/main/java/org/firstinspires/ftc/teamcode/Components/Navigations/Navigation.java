package org.firstinspires.ftc.teamcode.Components.Navigations;

public class Navigation {
    private static double xPosition;
    private static double yPosition;
    private static double angle;
    private static boolean inVuforia = false;

    public static synchronized void editDistance(double changeX, double changeY) {
        xPosition+=changeX;
        yPosition+=changeY;
    }

    public static void changeAngle(double newAngle) {
        angle = newAngle;
    }

    public static double getXposition(){
        return xPosition;
    }

    public static void setXposition(double newValue){
        xPosition=newValue;
    }

    public static double getYposition(){
        return yPosition;
    }

    public static void getYposition(double newValue){
        yPosition = newValue;
    }

    public static double getAngle() {
        return angle;
    }

    public static void setAngle(double newAngle) {
        angle = newAngle;
    }

    public static boolean getInVuforia() {
        return inVuforia;
    }

    public static boolean setInVuforia(boolean set) {
        inVuforia = set;
        return inVuforia;
    }

}
