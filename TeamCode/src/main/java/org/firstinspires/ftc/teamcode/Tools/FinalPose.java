package org.firstinspires.ftc.teamcode.Tools;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;

public class FinalPose extends Object{
    public static double x;
    public static double y;
    public static double Yaw;

    public static void setfinalPose(double X, double Y, double yaw){
        x = Mouse.getX();
        y = Mouse.getY();
        Yaw = Mouse.getTheta();
    }
    public static void poseUpdate() {
        Mouse.update();
        x = -Mouse.getX();
        y = -Mouse.getY();
        Yaw = Mouse.getTheta();

    }
    public static void Reset(){Mouse.configureOtos();}

}
