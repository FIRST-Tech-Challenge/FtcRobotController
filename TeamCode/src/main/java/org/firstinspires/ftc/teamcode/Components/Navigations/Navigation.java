package org.firstinspires.ftc.teamcode.Components.Navigations;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Navigations.Odometry;
import org.firstinspires.ftc.teamcode.Components.Navigations.VuforiaWebcam;

public class Navigation {
    private static double xPosition;
    private static double yPosition;
    private static double angle;
    private static boolean inVuforia = false;

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

    public static double getAngle() {
        return angle;
    }

    public double[] getPosition(){
        double pos[] = {xPosition,yPosition,angle};
        return pos;
    }

    public void setPosition(double x, double y, double newAngle){
        xPosition = x;
        yPosition = y;
        angle = newAngle;
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


    public void navigate( LinearOpMode op) {
        Thread vuforia = new Thread(new VuforiaWebcam(op));
        Thread odometry = new Thread(new Odometry(op));

        odometry.start();
        vuforia.start();
    }
}
