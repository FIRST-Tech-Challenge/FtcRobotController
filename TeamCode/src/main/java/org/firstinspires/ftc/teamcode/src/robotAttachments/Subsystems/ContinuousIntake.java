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

/**
 * this is the class for our robot's intake subsystem
 */
public class ContinuousIntake {
    /**
     * The power for going forward
     */
    final static double forwardPower = 1;
    /**
     * The Position servo must be to release an item
     */
    private static final double open = .4; // this position needs to be adjusted!
    /**
     * The Position servo must be to keep and item in the intake compartment
     */
    private static final double closed = .7; // this position needs to be adjusted
    /**
     * The item color sensor
     */
    public ColorRangeSensor colorSensor;
    /**
     * the distance sensor on the inside of the intake system, it is used to identify when an object enters the
     */
    public DistanceSensor distanceSensor;
    /**
     * DcMotor Object
     */
    DcMotor intakeMotor;
    /**
     * The internal Servo Object
     */
    Servo itemRelease;

    /**
     * Initializes from hardware map and names
     *
     * @param hardwareMap hardware map object
     * @param motorName   Name of intake motor
     * @param servoName   Name of lifting servo
     */
    public ContinuousIntake(HardwareMap hardwareMap, String motorName, String servoName, String distanceSensor, String colorSensor, boolean sensorDetectionLight) {
        intakeMotor = hardwareMap.dcMotor.get(motorName);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.distanceSensor = hardwareMap.get(DistanceSensor.class, distanceSensor);

        this.colorSensor = hardwareMap.get(ColorRangeSensor.class, colorSensor);
        this.colorSensor.enableLed(sensorDetectionLight);


        itemRelease = hardwareMap.servo.get(servoName);
    }

    public gameObject identifyContents() {
        return ContinuousIntake.gameObject.identify(this.getRGB());
    }

    /**
     * Initializes from hardware map and names
     * Initializes the Color Sensor
     *
     * @param hardwareMap          this is the hardware map
     * @param motorName            this is a string for the name of the intake motor
     * @param servoName            this is a string for the name of the intake motor
     * @param colorSensor          this is a string for the name of the color sensor on the intake
     * @param sensorDetectionLight this is a boolean for turning the color sensor light on(true) or off(false)
     */
    public ContinuousIntake(HardwareMap hardwareMap, String motorName, String servoName, String colorSensor, boolean sensorDetectionLight) {
        intakeMotor = hardwareMap.dcMotor.get(motorName);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.colorSensor = hardwareMap.get(ColorRangeSensor.class, colorSensor);
        this.colorSensor.enableLed(sensorDetectionLight);

        itemRelease = hardwareMap.servo.get(servoName);

    }

    /**
     * It treats color as 3D space and returns the distance between the two points
     *
     * @param sight  The first set of RGB values
     * @param object The second set of RGB values
     * @return The distance between the two, smaller is closer
     */
    public static double getDifferenceOfColor(double[] sight, double[] object) {
        double difference;
        double r = Math.abs(sight[0] - object[0]);
        double g = Math.abs(sight[1] - object[1]);
        double b = Math.abs(sight[2] - object[2]);
        // this calculates the 3d distance between colors
        difference = Math.sqrt(Math.pow(Math.sqrt(Math.pow(r, 2) + Math.pow(g, 2)), 2) + Math.pow(b, 2));
        return difference;
    }

    /**
     * this turns on the intake motor to intake freight
     */
    public void intakeOn() {
        intakeMotor.setPower(forwardPower);
    }

    /**
     * turns off the intake motor
     */
    public void intakeOff() {
        intakeMotor.setPower(0);
    }

    /**
     * reverses the intake motor to remove freight from the intake bucket
     */
    public void intakeReverse() {
        intakeMotor.setPower(-forwardPower);
    }

    /**
     * @param power a variable input for the power of the intake motor
     *              this sets the power of the intake motor to the power variable input
     */
    public void setMotorPower(double power) {
        intakeMotor.setPower(power);
    }

    /**
     * uses the intake's servo hinge to put the intake in the up position
     */
    public void setServoUp() {
        itemRelease.setPosition(open);
    }

    /**
     * uses the intake's servo hinge to put the intake in the down position
     */
    public void setServoDown() {
        itemRelease.setPosition(closed);
    }

    /**
     * this following method takes a parameter for the type of color and outputs the sensor's number for that color
     *
     * @param color the name of the color wanted as a String
     * @return this returns a number of the value for the name of the wanted color
     */
    public double getColor(String color) {
        HashMap<String, Double> colorKey = new HashMap<String, Double>() {{
            put("red", (double) (colorSensor.red()));
            put("green", (double) colorSensor.green());
            put("blue", (double) (colorSensor.blue()));
            put("alpha", (double) (colorSensor.red()));
        }};
        return colorKey.get(color).doubleValue();
    }

    /**
     * Returns what the Color Sensor Sees
     *
     * @return Returns values from 0 to 255 in the form of R,G,B
     */
    public double[] getRGB() {
        return new double[]{colorSensor.red(), colorSensor.green(), colorSensor.blue()};
    }

    /**
     * Gets how close the Object is to the sensor
     *
     * @return The distance in Inches
     */
    public double getSensorDistance() {
        double distance = distanceSensor.getDistance(DistanceUnit.CM);
        return distance;
    }

    /**
     * Analyzes the content of the bucket to determine shape, returns the corresponding blink pattern
     *
     * @return Returns the blink pattern for the object in the bucket
     */
    public RevBlinkinLedDriver.BlinkinPattern getLEDPatternFromFreight() {
        RevBlinkinLedDriver.BlinkinPattern o = gameObject.RevColorOfObj.get(ContinuousIntake.gameObject.identify(this.getRGB()));
        return o;
    }

    /**
     * A Enum for each object the bucket can pick up
     */
    public enum gameObject {
        BALL,
        CUBESMOOTH,
        CUBEWAFFLE,
        DUCK,
        EMPTY;

        /**
         * A list of every possible enum value
         */
        protected static final ArrayList<gameObject> gameObjectList = new ArrayList<gameObject>(Arrays.asList(gameObject.values()));

        /**
         * The Key is the game object, the value is what LED pattern it should corespond to
         */
        protected static final HashMap<gameObject, BlinkinPattern> RevColorOfObj = new HashMap<gameObject, BlinkinPattern>() {{
            put(gameObject.BALL, BlinkinPattern.WHITE);
            put(gameObject.CUBESMOOTH, BlinkinPattern.ORANGE);
            put(gameObject.CUBEWAFFLE, BlinkinPattern.ORANGE);
            put(gameObject.DUCK, BlinkinPattern.GREEN);
            put(gameObject.EMPTY, null);
        }};

        /**
         * The Key is the game object, the value is the RGB value of what the sensor sees
         */
        protected static final HashMap<gameObject, double[]> RGBOfObj = new HashMap<gameObject, double[]>() {{
            put(gameObject.BALL, new double[]{611.0, 652.0, 594.5});
            put(gameObject.CUBESMOOTH, new double[]{56.5, 34.5, 20});
            put(gameObject.CUBEWAFFLE, new double[]{31, 18.5, 12});
            put(gameObject.DUCK, new double[]{17.67, 15.67, 9});
            put(gameObject.EMPTY, new double[]{8, 9, 6});
        }};

        /**
         * Determines what game object best matches the color pattern provided
         *
         * @param RGB The color pattern in the form of RGB
         * @return Returns what game object the RGB best matches
         */
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

