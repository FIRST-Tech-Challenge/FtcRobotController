package org.firstinspires.ftc.teamcode.Susbsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotClass;

public class Drive {

    public static double[] wheelSpeedsMecanum = new double[4];
    public static double[] wheelSpeedsSixWheel = new double[4];

    public static double[] DriveCartesian(double rotX, double rotY, double rotation){

        wheelSpeedsMecanum[RobotClass.kFrontLeft] = (rotY + rotX + rotation);
        wheelSpeedsMecanum[RobotClass.kBackLeft] = (rotY - rotX + rotation);
        wheelSpeedsMecanum[RobotClass.kFrontRight] = (rotY - rotX - rotation);
        wheelSpeedsMecanum[RobotClass.kBackRight] = (rotY + rotX - rotation);
        return normalizeRanges(wheelSpeedsMecanum);
    }
    public static double[] DriveSixWheel(double forward, double turn){
        wheelSpeedsSixWheel[RobotClass.kFrontLeft] = (forward + turn);
        wheelSpeedsSixWheel[RobotClass.kBackLeft] = (forward + turn);
        wheelSpeedsSixWheel[RobotClass.kFrontRight] = (forward - turn);
        wheelSpeedsSixWheel[RobotClass.kBackRight] = (forward - turn);

        return normalizeRanges(wheelSpeedsSixWheel);
    }

    public static double[] getWheelSpeedsMecanum(){
        return wheelSpeedsMecanum;
    }
    public static double getWheelSpeed(int wheelId){
        return (int) wheelSpeedsMecanum[wheelId];
    }
    public static double[] correctDrift(double[] wheelSpeeds, Telemetry telemetry, double rotationRate){ //milliseconds
        rotationRate /= 2;
        if(rotationRate < -0.5){
            wheelSpeeds[RobotClass.kBackRight] *= (1 + rotationRate);
            wheelSpeeds[RobotClass.kBackLeft] *= (1 + rotationRate );
            wheelSpeeds[RobotClass.kFrontLeft] *= -(1 + rotationRate );
            wheelSpeeds[RobotClass.kFrontRight] *= -(1 + rotationRate );

        }
        if(rotationRate > 0.5){
            wheelSpeeds[RobotClass.kBackRight] *= -(1 + rotationRate);
            wheelSpeeds[RobotClass.kBackLeft] *= -(1 + rotationRate);
            wheelSpeeds[RobotClass.kFrontLeft] *= (1 + rotationRate);
            wheelSpeeds[RobotClass.kFrontRight] *= (1 + rotationRate);

        }

        telemetry.addData("changeInHeadingPerMillisecond", rotationRate);
        return wheelSpeeds;
    }
    public static double[] normalizeRanges(double[] wheelSpeeds){
        double maxWheelSpeed = Math.max(Math.max(wheelSpeeds[0], wheelSpeeds[1]), Math.max(wheelSpeeds[2], wheelSpeeds[3]));
        maxWheelSpeed = Math.max(maxWheelSpeed, 1);
        wheelSpeeds[0] /= maxWheelSpeed;
        wheelSpeeds[1] /= maxWheelSpeed;
        wheelSpeeds[2] /= maxWheelSpeed;
        wheelSpeeds[3] /= maxWheelSpeed;
        return wheelSpeeds;
    }

}
