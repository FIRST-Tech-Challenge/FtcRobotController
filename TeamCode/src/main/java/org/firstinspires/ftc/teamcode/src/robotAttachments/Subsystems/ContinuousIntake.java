package org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

public class ContinuousIntake {
    final static double forwardPower = 1;
    DcMotor intakeMotor;


    private static final double BucketUpPosition = .98;
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


    /*
    @Deprecated
    protected double[] getRGBFromList(ContinuousIntake.gameObject object) {
        //this is a list of approximations of values of RGB for different objects and empty
        //css is cube smooth side and cws is cube waffle side

        /* empty: 8,9,6
           cws:  31, 18.5, 12
           css:  56.5, 34.5, 20
           ball: 611, 652, 594.5
           duck: 17.67, 15.67, 9



        switch (object) {

            case BALL:
                // change the values of these arrays
                // order of color values in array is Red Green Blue
                double[] ball = {611, 652, 594.5};
                return ball;

            case CUBESMOOTH:
                double[] cubeSmooth = {56.5, 34.5, 20};
                return cubeSmooth;

            case CUBEWAFFLE:
                double[] cubeWaffle = {31, 18.5, 12};
                return cubeWaffle;

            case DUCK:
                double[] duck = {17.67, 15.67, 9};
                return duck;
            case EMPTY:
                return new double[]{8, 9, 6};

        }
        return new double[]{0, 0, 0};
    }*/

    public static double getDifferenceOfColor(double[] sight, double[] object) {
        double difference;
        double r = Math.abs(sight[0] - object[0]);
        double g = Math.abs(sight[1] - object[1]);
        double b = Math.abs(sight[2] - object[2]);
        // this calculates the 3d distance between colors
        difference = Math.sqrt(Math.pow(Math.sqrt(Math.pow(r, 2) + Math.pow(g, 2)), 2) + Math.pow(b, 2));
        return difference;
    }

    public RevBlinkinLedDriver.BlinkinPattern getLEDPatternFromFreight() {
        RevBlinkinLedDriver.BlinkinPattern o = gameObject.RevColorOfObj.get(ContinuousIntake.gameObject.identify(this.getRGB()));
        return o;
    }



    /*
    @Deprecated
    private RevBlinkinLedDriver.BlinkinPattern getRevColorFromEnum(gameObject o) {
        switch (o) {
            case BALL:
                return RevBlinkinLedDriver.BlinkinPattern.WHITE;
            case CUBESMOOTH:
                return RevBlinkinLedDriver.BlinkinPattern.ORANGE;
            case CUBEWAFFLE:
                return RevBlinkinLedDriver.BlinkinPattern.ORANGE;
            case DUCK:
                return RevBlinkinLedDriver.BlinkinPattern.GREEN;
            case EMPTY:
                return RevBlinkinLedDriver.BlinkinPattern.BLACK;
        }
        return RevBlinkinLedDriver.BlinkinPattern.BLACK;
    }
     */

    public enum gameObject {
        BALL,
        CUBESMOOTH,
        CUBEWAFFLE,
        DUCK,
        EMPTY;
        protected static final ArrayList<gameObject> gameObjectList = new ArrayList<gameObject>(Arrays.asList(gameObject.values()));

        protected static final HashMap<gameObject, BlinkinPattern> RevColorOfObj = new HashMap<gameObject, BlinkinPattern>() {{
            put(gameObject.BALL, BlinkinPattern.WHITE);
            put(gameObject.CUBESMOOTH, BlinkinPattern.ORANGE);
            put(gameObject.CUBEWAFFLE, BlinkinPattern.ORANGE);
            put(gameObject.DUCK, BlinkinPattern.GREEN);
            put(gameObject.EMPTY, null);
        }};

        protected static final HashMap<gameObject, double[]> RGBOfObj = new HashMap<gameObject, double[]>() {{
            put(gameObject.BALL, new double[]{611.0, 652.0, 594.5});
            put(gameObject.CUBESMOOTH, new double[]{56.5, 34.5, 20});
            put(gameObject.CUBEWAFFLE, new double[]{31, 18.5, 12});
            put(gameObject.DUCK, new double[]{17.67, 15.67, 9});
            put(gameObject.EMPTY, new double[]{8, 9, 6});
        }};

        public static gameObject identify(double[] RGB) {
            int size = gameObject.gameObjectList.size();
            double[][] originalRGB = new double[size][3];
            double[] differences = new double[size];
            for (int x = 0; x < size; x++) {
                originalRGB[x] = gameObject.RGBOfObj.get((gameObject.gameObjectList.get(x)));
            }
            for (int x = 0; x < size; x++) {
                differences[x] = getDifferenceOfColor(RGB, originalRGB[x]);
            }
            double smallestValue = Double.MAX_VALUE;
            int index = -1;

            for (int i = 0; i < size; i++) {
                if (differences[i] < smallestValue) {
                    smallestValue = differences[i];
                    index = i;
                }
            }
            return gameObject.gameObjectList.get(index);


        }

    }

}

