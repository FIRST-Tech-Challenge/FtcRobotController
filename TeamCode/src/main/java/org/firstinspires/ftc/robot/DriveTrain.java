package org.firstinspires.ftc.robot;

import com.arcrobotics.ftclib.hardware.motors.Motor;

public class DriveTrain {

    public Motor driveLeft, driveRight;

    public DriveTrain(Motor driveLeft, Motor driveRight) {
        this.driveLeft = driveLeft;
        this.driveRight = driveRight;

        driveLeft.setRunMode(Motor.RunMode.VelocityControl);
        driveRight.setRunMode(Motor.RunMode.VelocityControl);
//        driveLeft.setInverted(true);
        driveLeft.setVeloCoefficients(0.05, 0, 0);
        driveRight.setVeloCoefficients(0.05, 0, 0);
        driveLeft.setPositionCoefficient(0.05);
        driveRight.setPositionCoefficient(0.05);
    }

    public DriveTrain(Motor driveLeft, Motor driveRight, Motor.RunMode runMode) {
        this(driveLeft, driveRight);

        this.driveLeft.setRunMode(runMode);
        this.driveRight.setRunMode(runMode);
    }

    public void setSpeed(double leftSpeed, double rightSpeed) {
        driveLeft.set(-leftSpeed);
        driveRight.set(rightSpeed);
    }

    public void setTargetPosition(int targetPosition) {
        driveLeft.setTargetPosition(-targetPosition);
        driveLeft.setTargetPosition(targetPosition);
    }

    public void setTargetPosition(int leftTargetPosition, int rightTargetPosition) {
        driveLeft.setTargetPosition(-leftTargetPosition);
        driveLeft.setTargetPosition(rightTargetPosition);
    }

    public int[] getPosition() {
        return new int[]{driveLeft.getCurrentPosition(), driveRight.getCurrentPosition()};
    }

    public double[] getDistance() {
        return new double[]{driveLeft.getDistance(), driveRight.getDistance()};
    }

    public double[] getRevolutions() {
        return new double[]{driveLeft.encoder.getRevolutions(), driveRight.encoder.getRevolutions()};
    }

    public void resetEncoders() {
        driveLeft.resetEncoder();
        driveRight.resetEncoder();
    }



}
