package org.firstinspires.ftc.teamcode;

public class VelocityPID {
    double leftBackResult(double encoderPos, double oldPos) {
        double result = (encoderPos - oldPos);
        return result;
    }
}