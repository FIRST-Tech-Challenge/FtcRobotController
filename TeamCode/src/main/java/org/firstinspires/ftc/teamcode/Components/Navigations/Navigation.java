package org.firstinspires.ftc.teamcode.Components.Navigations;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Navigation {
    private static double xPosition;
    private static double yPosition;
    private static float angle;
    private static boolean inVuforia = false;
    Thread odometry = null;
    Thread vuforia=null;
    public Navigation(LinearOpMode OpMode){
        odometry = new Thread(new Odometry(OpMode));
    }

    public static synchronized void editDistance(double changeX, double changeY) {
        xPosition+=changeX;
        yPosition+=changeY;
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

    public static void setYposition(double newValue){
        yPosition = newValue;
    }

    public static float getAngle() {
        return angle;
    }

    public double[] getPosition(){
        return new double[]{xPosition,yPosition,angle};
    }

    public static void setPosition(double x, double y, float newAngle){
        xPosition = x;
        yPosition = y;
        angle = newAngle;
    }

    public static void setAngle(float newAngle) {
        angle = newAngle;
    }

    public static boolean getInVuforia() {
        return inVuforia;
    }

    public static boolean setInVuforia(boolean set) {
        inVuforia = set;
        return inVuforia;
    }


    public void navigate( LinearOpMode op) {
        //Thread vuforia = new Thread(new VuforiaWebcam(op));
        odometry.start();
        //vuforia.start();
    }
    public void navigateTeleOp( LinearOpMode op) {
        odometry.start();
        vuforia = new Thread(new VuforiaWebcam(op));
        vuforia.start();
    }
}
