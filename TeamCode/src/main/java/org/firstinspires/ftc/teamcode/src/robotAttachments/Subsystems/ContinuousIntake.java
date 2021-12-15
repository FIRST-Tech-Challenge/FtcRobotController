package org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems;


import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ContinuousIntake {
    final static double forwardPower = 1;
    DcMotor intakeMotor;

    private static final double BucketUpPosition = .92;
    private static final double BucketDownPosition = .46;
    Servo slantServo;
    public ColorRangeSensor intakeSensor;
    public DistanceSensor intakeSensor_D;


    public ContinuousIntake(HardwareMap hardwareMap, String motorName, String servoName) {
        intakeMotor = hardwareMap.dcMotor.get(motorName);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slantServo = hardwareMap.servo.get(servoName);

    }

    /*
    This next constructor has a parameter for the name of the color sensor in the configuration
    and a boolean value for to have the color sensor LED on or off
     */
    public ContinuousIntake(HardwareMap hardwareMap, String motorName, String servoName, String colorSensor, boolean sensorDetectionLight) {
        intakeMotor = hardwareMap.dcMotor.get(motorName);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeSensor = hardwareMap.get(ColorRangeSensor.class, colorSensor);
        //intakeSensor_D = hardwareMap.get(DistanceSensor.class, colorSensor);
        intakeSensor.enableLed(sensorDetectionLight);

        slantServo = hardwareMap.servo.get(servoName);

    }

    public void intakeOn() {
        intakeMotor.setPower(forwardPower);
    }

    public void intakeOff() {
        intakeMotor.setPower(0);
    }

    public void intakeReverse() {
        intakeMotor.setPower(-forwardPower);
    }

    public void setMotorPower(double power) {
        intakeMotor.setPower(power);
    }

    public void setServoUp() {
        slantServo.setPosition(BucketUpPosition);
    }

    public void setServoDown() {
        slantServo.setPosition(BucketDownPosition);
    }


    /*
    this following method takes a parameter for the type of color and outputs the
    sensor's number for that color
     */
    public double getColor(String color) {
        int i;
        double[] colorNumber = {intakeSensor.red(), intakeSensor.blue(), intakeSensor.green(), intakeSensor.alpha(), intakeSensor.argb()};
        switch (color) {
            case "red":
                i = 0;
                break;
            case "blue":
                i = 1;
                break;
            case "green":
                i = 2;
                break;
            case "alpha":
                i = 3;
                break;
            case "argb":
                i = 4;
                break;
            default:
                i = 0;
        }
        return colorNumber[i];

    }

    public double[] getRGB() {

        double[] colorNumber = {intakeSensor.red(), intakeSensor.green(), intakeSensor.blue()};
        return colorNumber;

    }

    public double getCloseDistance() {
        double distance = intakeSensor.getDistance(DistanceUnit.INCH);
        return distance;
    }

    public double linearEquation(double m, double b, double x) {
        double y = (m * x) + b;
        return y;
    }

    protected double[] getRGBFromList(ContinuousIntake.gameObject object) {

        switch (object) {

            case BALL:
                // change the values of these arrays
                // order of color values in array is Red Green Blue
                double[] ball = {1, 1, 1};
                return ball;

            case CUBESMOOTH:
                double[] cubeSmooth = {1, 1, 1};
                return cubeSmooth;

            case CUBEWAFFLE:
                double[] cubeWaffle = {1, 1, 1};
                return cubeWaffle;

            case DUCK:
                double[] duck = {1, 1, 1};
                return duck;
            case EMPTY:
                return new double[]{1, 1, 1};

        }
        return new double[]{0, 0, 0};
    }

    public double getDifferenceOfColor(double[] sight, double[] object) {
        double difference;
        double r = Math.abs(sight[0] - object[0]);
        double g = Math.abs(sight[1] - object[1]);
        double b = Math.abs(sight[2] - object[2]);
        // this calculates the 3d distance between colors

        difference = Math.sqrt(Math.sqrt(Math.pow(r, 2) + Math.pow(g, 2)) + Math.pow(b, 2));

        return difference;
    }

    public String identify(double[] sight) {
        String type = "uknown";
        double distanceFromBall = getDifferenceOfColor(sight, getRGBFromList(gameObject.BALL));
        double distanceFromCubeSmooth = getDifferenceOfColor(sight, getRGBFromList(gameObject.CUBESMOOTH));
        double distanceFromCubeWaffle = getDifferenceOfColor(sight, getRGBFromList(gameObject.CUBEWAFFLE));
        double distanceFromDuck = getDifferenceOfColor(sight, getRGBFromList(gameObject.DUCK));
        double distanceFromEmpty = getDifferenceOfColor(sight, getRGBFromList(gameObject.EMPTY));
        // determining the type of object involves comparing the 3D color distance from all known color positions
        // this is what the following logic statements determine
        if (distanceFromEmpty > distanceFromDuck && distanceFromEmpty > distanceFromBall && distanceFromEmpty > distanceFromCubeSmooth && distanceFromEmpty > distanceFromCubeWaffle) {
            if (distanceFromBall < distanceFromDuck && distanceFromBall < distanceFromCubeSmooth && distanceFromBall < distanceFromCubeWaffle) {
                type = "ball";
            } else if (distanceFromDuck < distanceFromCubeSmooth && distanceFromDuck < distanceFromCubeWaffle) {
                type = "duck";
            } else {
                type = "cube";
            }

        } else {
            type = "empty";
        }
        return type;

    }

    public enum gameObject {
        BALL,
        CUBESMOOTH,
        CUBEWAFFLE,
        DUCK,
        EMPTY

    }


}

