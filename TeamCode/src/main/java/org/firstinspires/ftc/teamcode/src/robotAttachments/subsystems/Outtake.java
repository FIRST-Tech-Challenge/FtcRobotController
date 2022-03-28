package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.src.utills.Controllable;
import org.firstinspires.ftc.teamcode.src.utills.enums.FreightFrenzyGameObject;
import org.firstinspires.ftc.teamcode.src.utills.enums.RGBCameraColors;

/**
 * this is the class for our robot's intake subsystem
 */
public class Outtake implements Controllable {
    /**
     * The Position servo must be to release an item
     */
    private static final double open = .8; // this position needs to be adjusted!

    /**
     * The Position servo must be to keep and item in the intake compartment
     */
    private static final double closed = 0.45; // this position needs to be adjusted

    /**
     * The item color sensor
     */
    private final ColorRangeSensor colorSensor;

    /**
     * The internal Servo Object
     */
    private final Servo itemRelease;
    private final ElapsedTime yTimer = new ElapsedTime();
    /**
     * A boolean that tells if the servo is closed or opened
     */
    private boolean isClosed;
    private boolean y_depressed2 = true;


    /**
     * Initializes from hardware map and names
     *
     * @param hardwareMap          hardware map object
     * @param servoName            Name of lifting servo
     * @param colorSensor          name of the color sensor
     * @param sensorDetectionLight true if the light should be on, false if the light should be off
     */
    public Outtake(HardwareMap hardwareMap, String servoName, String colorSensor, boolean sensorDetectionLight) {
        this.colorSensor = hardwareMap.get(ColorRangeSensor.class, colorSensor);
        this.colorSensor.enableLed(sensorDetectionLight);

        this.itemRelease = hardwareMap.servo.get(servoName);
        this.setServoClosed();
    }

    /**
     * Identifies the contents in the bucket
     *
     * @return The {@link FreightFrenzyGameObject} inside the bucket
     */
    public FreightFrenzyGameObject identifyContents() {
        return FreightFrenzyGameObject.identify(this.getRGB());
    }

    /**
     * A getter for the isClosed boolean
     *
     * @return Returns true if the grabber is closed, false if otherwise
     */
    public boolean isClosed() {
        return this.isClosed;
    }


    /**
     * uses the intake's servo hinge to put the intake in the up position
     */
    public void setServoOpen() {
        itemRelease.setPosition(open);
        isClosed = false;
    }

    /**
     * uses the intake's servo hinge to put the intake in the down position
     */
    public void setServoClosed() {
        itemRelease.setPosition(closed);
        isClosed = true;
    }

    public void setServoPos(double pos) {
        itemRelease.setPosition(pos);
        isClosed = false;
    }

    /**
     * this following method takes a parameter for the type of color and outputs the sensor's number for that color
     *
     * @param color the name of the color wanted
     * @return this returns a number of the value for the name of the wanted color
     */
    public int getColor(RGBCameraColors color) {
        switch (color) {
            case Red:
                return colorSensor.red();

            case Blue:
                return colorSensor.blue();

            case Green:
                return colorSensor.green();

            case Alpha:
                return colorSensor.alpha();
            default:
                return 0;
        }
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
        return colorSensor.getDistance(DistanceUnit.CM);
    }

    /**
     * @return returns a true or false value of whether or not an item passes the intake distance sensor
     */
    public boolean itemInBucket() {
        return (this.getSensorDistance() < 9);
    }

    /**
     * Analyzes the content of the bucket to determine shape, returns the corresponding blink pattern
     *
     * @return Returns the blink pattern for the object in the bucket
     */
    public RevBlinkinLedDriver.BlinkinPattern getLEDPatternFromFreight() {
        return FreightFrenzyGameObject.getLEDColorFromItem(FreightFrenzyGameObject.identify(this.getRGB()));
    }

    /**
     * Controls the intake and outtake
     *
     * @param gamepad1 The first gamepad
     * @param gamepad2 The second gamepad
     * @return The current item in the intake
     */
    @Override
    public Object gamepadControl(Gamepad gamepad1, Gamepad gamepad2) {

        FreightFrenzyGameObject currentObject = FreightFrenzyGameObject.EMPTY; // Assigning to null so the compiler is happy


        //Out take controls
        {
            if (!gamepad2.y) {
                y_depressed2 = true;
            }
            if (gamepad2.y && y_depressed2) {
                y_depressed2 = false;
                if (this.isClosed()) {
                    this.setServoOpen();
                    this.yTimer.reset();
                } else {
                    this.setServoClosed();
                }
                currentObject = FreightFrenzyGameObject.EMPTY;
            }

            if (this.yTimer.seconds() > 1.25) {
                this.setServoClosed();
            }
        }

        return (Object) currentObject;

    }
}

