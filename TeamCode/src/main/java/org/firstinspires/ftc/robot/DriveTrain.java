package org.firstinspires.ftc.robot;

import com.arcrobotics.ftclib.hardware.motors.Motor;

public class DriveTrain {

    public Motor driveLeft, driveRight;

    public DriveTrain(Motor driveLeft, Motor driveRight) {
        this.driveLeft = driveLeft;
        this.driveRight = driveRight;

        driveLeft.setRunMode(Motor.RunMode.VelocityControl);
        driveRight.setRunMode(Motor.RunMode.VelocityControl);
        driveLeft.setInverted(true);
        driveLeft.setVeloCoefficients(0.05, 0, 0);
        driveRight.setVeloCoefficients(0.05, 0, 0);
    }

    public void setSpeed(double leftSpeed, double rightSpeed) {
        driveLeft.set(leftSpeed);
        driveRight.set(rightSpeed);
    }

    public int[] getEncoderCounts() {
        return new int[]{driveLeft.getCurrentPosition(), driveRight.getCurrentPosition()};
    }



}
