package org.firstinspires.ftc.teamcode.Susbsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotClass;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class Drive {

    public static double[] wheelSpeedsMecanum = new double[4];
    public static double[] wheelSpeedsSixWheel = new double[4];

    public static double[] DriveCartesian(double rotX, double rotY, double rotation){
//        driveMotors[RobotClass.kFrontLeft].setPower((rotY + rotX - turn) / denominator);
//        driveMotors[RobotClass.kBackLeft].setPower((rotY - rotX - turn) / denominator);
//        driveMotors[RobotClass.kBackRight].setPower((rotY - rotX + turn) / denominator);
//        driveMotors[RobotClass.kFrontRight].setPower((rotY + rotX + turn) / denominator);

        wheelSpeedsMecanum[RobotClass.kFrontLeft] = (rotY + rotX - rotation);
        wheelSpeedsMecanum[RobotClass.kBackLeft] = (rotY - rotX - rotation);
        wheelSpeedsMecanum[RobotClass.kBackRight] = (rotY - rotX + rotation);
        wheelSpeedsMecanum[RobotClass.kFrontRight] = (rotY + rotX + rotation);
        //return normalizeRanges(wheelSpeedsMecanum);
        return wheelSpeedsMecanum;
    }
    public static double[] DriveSixWheel(double forward, double turn){
        wheelSpeedsSixWheel[RobotClass.kFrontLeft] = (forward + turn);
        wheelSpeedsSixWheel[RobotClass.kBackLeft] = (forward + turn);
        wheelSpeedsSixWheel[RobotClass.kFrontRight] = (forward - turn);
        wheelSpeedsSixWheel[RobotClass.kBackRight] = (forward - turn);
        return wheelSpeedsSixWheel;
        //return normalizeRanges(wheelSpeedsSixWheel);
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
            wheelSpeeds[RobotClass.kBackLeft] *= -(1 + rotationRate );
            wheelSpeeds[RobotClass.kFrontLeft] *= -(1 + rotationRate );
            wheelSpeeds[RobotClass.kFrontRight] *= (1 + rotationRate );

        }
        if(rotationRate > 0.5){
            wheelSpeeds[RobotClass.kBackRight] *= -(1 + rotationRate);
            wheelSpeeds[RobotClass.kBackLeft] *= (1 + rotationRate);
            wheelSpeeds[RobotClass.kFrontLeft] *= (1 + rotationRate);
            wheelSpeeds[RobotClass.kFrontRight] *= -(1 + rotationRate);

        }

        telemetry.addData("changeInHeadingPerMillisecond", rotationRate);
        return wheelSpeeds;
        //return normalizeRanges(wheelSpeeds);
    }
    public static double[] normalizeRanges(double[] wheelSpeeds, double x, double y, double rX){

        double maxWheelSpeed = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rX), 1);

        maxWheelSpeed = Math.max(maxWheelSpeed, 1);
        wheelSpeeds[0] /= maxWheelSpeed;
        wheelSpeeds[1] /= maxWheelSpeed;
        wheelSpeeds[2] /= maxWheelSpeed;
        wheelSpeeds[3] /= maxWheelSpeed;
        return wheelSpeeds;
    }

}
