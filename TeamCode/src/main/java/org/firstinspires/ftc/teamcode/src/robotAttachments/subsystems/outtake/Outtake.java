package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.outtake;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;

import org.firstinspires.ftc.teamcode.src.utills.Controllable;
import org.firstinspires.ftc.teamcode.src.utills.enums.FreightFrenzyGameObject;
import org.firstinspires.ftc.teamcode.src.utills.enums.RGBCameraColors;

public interface Outtake extends Controllable<FreightFrenzyGameObject> {
    /**
     * Identifies the contents in the bucket
     *
     * @return The {@link FreightFrenzyGameObject} inside the bucket
     */
    FreightFrenzyGameObject identifyContents();

    /**
     * Returns true if the outtake is closed
     *
     * @return Returns true if the grabber is closed, false if otherwise
     */
    boolean isClosed();


    /**
     * Opens the outtake
     */
    void open() throws InterruptedException;

    /**
     * Closes the intake
     */
    void close() throws InterruptedException;

    /**
     * Goes to the requested position
     *
     * @param pos The position to go to
     * @throws InterruptedException Throws if stop is requested during this time
     */
    void goTo(double pos) throws InterruptedException;

    /**
     * this following method takes a parameter for the type of color and outputs the sensor's number for that color
     *
     * @param color the name of the color wanted
     * @return this returns a number of the value for the name of the wanted color
     */
    int getColor(RGBCameraColors color);

    /**
     * Returns what the Color Sensor Sees
     *
     * @return Returns values from 0 to 255 in the form of R,G,B
     */
    double[] getRGB();

    /**
     * @return returns a true or false value of whether or not an item is in the intake
     */
    boolean itemInBucket();

    /**
     * Analyzes the content of the bucket to determine shape, returns the corresponding blink pattern
     *
     * @return Returns the blink pattern for the object in the bucket
     */
    BlinkinPattern getLEDPatternFromFreight();


}
