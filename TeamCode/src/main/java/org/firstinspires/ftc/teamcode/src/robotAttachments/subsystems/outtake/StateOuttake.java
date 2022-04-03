package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.outtake;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.src.utills.Controllable;
import org.firstinspires.ftc.teamcode.src.utills.enums.FreightFrenzyStateObject;

public class StateOuttake implements Controllable<FreightFrenzyStateObject> {
    /**
     * The item color sensor
     */
    private final ColorRangeSensor colorSensor;


    /**
     * The Position servo must be to release an item
     */
    private static final double open = .47; // this position needs to be adjusted!
    /**
     * The Position servo must be to keep and item in the intake compartment
     */
    private static final double closed = .7; // this position needs to be adjusted

    /**
     * The internal Servo Object
     */
    private final Servo itemRelease;
    private boolean isClosed;

    public StateOuttake(HardwareMap hardwareMap, String colorSensor, String servoName, boolean sensorDetectionLight) {
        this.colorSensor = hardwareMap.get(ColorRangeSensor.class, colorSensor);
        this.colorSensor.enableLed(sensorDetectionLight);


        itemRelease = hardwareMap.servo.get(servoName);
        this.setServoClosed();
        isClosed = true;
    }

    boolean y_depressed2 = true;

    private final ElapsedTime yTimer = new ElapsedTime();


    @Nullable
    @Override
    public FreightFrenzyStateObject gamepadControl(@NonNull Gamepad gamepad1, @NonNull Gamepad gamepad2) {
        if (!gamepad2.y) {
            y_depressed2 = true;
        }
        if (gamepad2.y && y_depressed2) {
            y_depressed2 = false;
            if (this.isClosed()) {
                this.setServoOpen();
                yTimer.reset();
            } else {
                this.setServoClosed();
            }
        }

        if (yTimer.seconds() > 1.25) {
            this.setServoClosed();
        }

        return this.identifyContents();
    }

    @Override
    public void halt() {
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

    /**
     * Analyzes the content of the bucket to determine shape, returns the corresponding blink pattern
     *
     * @return Returns the blink pattern for the object in the bucket
     */
    public RevBlinkinLedDriver.BlinkinPattern getLEDPatternFromFreight() {
        return FreightFrenzyStateObject.getLEDColorFromItem(FreightFrenzyStateObject.identify(this.getRGB()));
    }

    /**
     * Identifies the contents in the bucket
     *
     * @return The {@link FreightFrenzyStateObject} inside the bucket
     */
    public FreightFrenzyStateObject identifyContents() {
        return FreightFrenzyStateObject.identify(this.getRGB());
    }

    /**
     * Returns what the Color Sensor Sees
     *
     * @return Returns values from 0 to 255 in the form of R,G,B
     */
    public double[] getRGB() {
        return new double[]{colorSensor.red(), colorSensor.green(), colorSensor.blue()};
    }


}
