package org.firstinspires.ftc.teamcode.Susbsystem;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.AutoControl;
import org.firstinspires.ftc.teamcode.RobotClass;

import java.util.HashMap;
import java.util.Map;

public class AutoUtils {
    RobotClass robot;
    AutoControl autoControl = new AutoControl();
    boolean stopRequested = false;
    SparkFunOTOS opticalSensor;

    public static final int WHEEL_RADIUS = 2;
    public static final int CPR_OUTPUT_SHAFT_20TO1 = 560;
    public static final double WHEEL_CIRCUMFERENCE_INCH = 2 * Math.PI * WHEEL_RADIUS;
    public static final double TICKS_PER_INCH = CPR_OUTPUT_SHAFT_20TO1 /  WHEEL_CIRCUMFERENCE_INCH;

    public AutoUtils(RobotClass robot){
        this.robot = robot;
        opticalSensor = robot.opticalSensor;
    }
    Map<RobotClass.MOTORS, Double> wheelSpeeds = new HashMap<>();

    public void moveToPosition(double x, double y){

    }

    private final double kD = .01;
    @SuppressLint("DefaultLocale")
    public void AutoDrive(double targetDistance_INCH, double angle){

        robot.stopAndReset();
        while(robot.driveMotors.get(RobotClass.MOTORS.FRONT_LEFT).isBusy()){
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }


        angle = Math.toRadians(angle);
        //angle -= robot.getHeading();

        double maxErrorAllowed = 1.1 * TICKS_PER_INCH;
        double kP = 0.5;
        double kI = 0.01;
        double integralSum = 0;
        double error, correction;
        double targetDistance = targetDistance_INCH * TICKS_PER_INCH;

        while(!stopRequested){
            //SparkFunOTOS.Pose2D Pose2D = opticalSensor.getPos();

            error = targetDistance - (robot.driveMotors.get(RobotClass.MOTORS.FRONT_LEFT).getCurrentPosition());
            //(targetDistance - Math.sqrt(Math.pow((targetDistance * Math.cos(angle)
            //- Pose2D.x), 2) + (Math.pow((targetDistance * Math.sin(angle)) - Pose2D.y, 2))));

            //correction = kP * error + kI * integralSum + kD * getDerivative(error, targetDistance);
            if(Math.abs(error) < maxErrorAllowed) {
                break;
            }

            double x = Math.cos(angle);
            double y = Math.sin(angle);

            double rotY = y;
            double rotX = x;
            if(error < 0){
                rotY = -rotY;
                rotX = -rotX;
            }

            double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY), 1.0);

            wheelSpeeds.put(RobotClass.MOTORS.FRONT_LEFT, (rotY + rotX) / 2.0 / denominator );
            wheelSpeeds.put(RobotClass.MOTORS.FRONT_RIGHT, (rotY - rotX) / 2.0 / denominator);
            wheelSpeeds.put(RobotClass.MOTORS.BACK_LEFT, (rotY - rotX) / 2.0 / denominator);
            wheelSpeeds.put(RobotClass.MOTORS.BACK_RIGHT, (rotY + rotX) / 2.0 / denominator);


//            telemetry.addData("rotY", rotY);
//            telemetry.addData("rotX", rotX);
//            telemetry.addData("denominator", denominator);
//            telemetry.addData("correction", 0);// correction);
//            telemetry.addData("error", error);
//            telemetry.addLine(String.format("wheelSpeeds %6.4f %6.4f %6.4f %6.4f", wheelSpeeds.get(RobotClass.MOTORS.FRONT_LEFT), wheelSpeeds.get(RobotClass.MOTORS.FRONT_RIGHT), wheelSpeeds.get(RobotClass.MOTORS.BACK_RIGHT), wheelSpeeds.get(RobotClass.MOTORS.BACK_LEFT)));
//            telemetry.addData("targetDistance", targetDistance);
//            telemetry.addData("maxErrorAllowed", maxErrorAllowed);
//            telemetry.addData("MotorPos", robot.driveMotors.get(RobotClass.MOTORS.FRONT_LEFT).getCurrentPosition());
//            telemetry.update();

            //Drive.normalizeRanges(wheelSpeeds);

            UpdateWheelPowers();
        }

    }

    public void AutoTurn(double targetAngle){

        boolean atTarget = false;
        double angularDistance = 0;

        double initialAngle = Math.toDegrees(robot.getHeading());
        if(initialAngle < 0){
            initialAngle += 360;
        }

        do{

            int turnVal = 1;
            if(targetAngle - initialAngle < 0) turnVal = -1; // checks which way it should turn
            double currentAngle = Math.toDegrees(robot.getHeading());
            if(currentAngle < 0){
                currentAngle += 360;
            }
            angularDistance = Math.abs(currentAngle - targetAngle);

            if(angularDistance > 360){ // dealing with edge case
                turnVal = -1;
                angularDistance = 360 - angularDistance;
            }

            double powerReduce = angularDistance / 90;

            powerReduce = Math.max(powerReduce, 0.2);
            powerReduce = Math.min(powerReduce, 0.9);

            wheelSpeeds.put(RobotClass.MOTORS.FRONT_LEFT, -turnVal * powerReduce);
            wheelSpeeds.put(RobotClass.MOTORS.BACK_LEFT, -turnVal * powerReduce);
            wheelSpeeds.put(RobotClass.MOTORS.BACK_RIGHT, turnVal * powerReduce);
            wheelSpeeds.put(RobotClass.MOTORS.FRONT_RIGHT, turnVal * powerReduce);

//            telemetry.addData("angularDistance", angularDistance);
//            telemetry.addData("turnVal", turnVal);
//            telemetry.addData("currentAngle", currentAngle);
//            telemetry.addData("powerReduce", powerReduce);
//            telemetry.update();

            UpdateWheelPowers();

            if(angularDistance < 0.5) atTarget = true;
        }
        while(!atTarget && !stopRequested);
        simplePower(0);
    }
    public void simplePower(double power){
        wheelSpeeds.put(RobotClass.MOTORS.FRONT_LEFT, power);
        wheelSpeeds.put(RobotClass.MOTORS.BACK_LEFT, power);
        wheelSpeeds.put(RobotClass.MOTORS.BACK_RIGHT, power);
        wheelSpeeds.put(RobotClass.MOTORS.FRONT_RIGHT, power);
        UpdateWheelPowers();
    }

    public void stopMotors(){
        wheelSpeeds.put(RobotClass.MOTORS.FRONT_LEFT, 0.0);
        wheelSpeeds.put(RobotClass.MOTORS.BACK_LEFT, 0.0);
        wheelSpeeds.put(RobotClass.MOTORS.BACK_RIGHT, 0.0);
        wheelSpeeds.put(RobotClass.MOTORS.FRONT_RIGHT, 0.0);
        UpdateWheelPowers();
//        try {
//            Thread.sleep(500);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//        stop();
    }

    private double getDerivative(double error, double targetDistance){
        double previousError = error;
        error = targetDistance - robot.driveMotors.get(RobotClass.MOTORS.FRONT_LEFT).getCurrentPosition();
        return(kD * error ) / (previousError + 0.01);
    }
    public void UpdateWheelPowers(){
        robot.driveMotors.get(RobotClass.MOTORS.FRONT_LEFT).setPower(wheelSpeeds.get(RobotClass.MOTORS.FRONT_LEFT));
        robot.driveMotors.get(RobotClass.MOTORS.FRONT_RIGHT).setPower(wheelSpeeds.get(RobotClass.MOTORS.FRONT_RIGHT));
        robot.driveMotors.get(RobotClass.MOTORS.BACK_LEFT).setPower(wheelSpeeds.get(RobotClass.MOTORS.BACK_LEFT));
        robot.driveMotors.get(RobotClass.MOTORS.BACK_RIGHT).setPower(wheelSpeeds.get(RobotClass.MOTORS.BACK_RIGHT));
    }
}
