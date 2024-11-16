package org.firstinspires.ftc.teamcode.lm2COMPCODE.Teleop.packages;

public class ClawServo {
    public static void setServoPosition(double servoPosition) {
        ClawServo.servoPosition = servoPosition;
    }

    public static double getServoPosition() {
        return servoPosition;
    }
}