package org.firstinspires.ftc.teamcode;

public class helper {

    public static double getRPM(double velocityDegrees, double ratio){
        return velocityDegrees * 60 / 28 / ratio;
    }
    public static double getRPM(double velocityDegrees){
        return velocityDegrees * 60 / 28 / 1;
    }

}
