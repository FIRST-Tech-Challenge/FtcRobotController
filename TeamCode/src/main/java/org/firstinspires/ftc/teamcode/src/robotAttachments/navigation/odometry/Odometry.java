package org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.odometry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.LocalizationAlgorithm;
import org.firstinspires.ftc.teamcode.src.utills.ThreadedSubsystemInterface;

public interface Odometry extends LocalizationAlgorithm, ThreadedSubsystemInterface {


    /**
     * Returns the ports the encoders are plugged into for debug purposes
     *
     * @return the ports in the form Vertical Right, Vertical Left, Horizontal
     */
    int[] getPorts();

    /**
     * Outputs the location to the telemetry
     *
     * @param telemetry The OpMode telemetry
     */
    void showPosition(Telemetry telemetry);

    /**
     * Getter for the COUNTS_PER_INCH variable
     *
     * @return COUNTS_PER_INCH in ticks per inch
     */
    double getCOUNTS_PER_INCH();

    /**
     * Getter for the right encoder position
     *
     * @return The right encoder position in ticks
     */
    int returnRightEncoderPosition();

    /**
     * Getter for the left encoder position
     *
     * @return The left encoder position in ticks
     */
    int returnLeftEncoderPosition();

    /**
     * Getter for the horizontal encoder position
     *
     * @return The horizontal encoder position in ticks
     */
    int returnHorizontalEncoderPosition();

    /**
     * Returns the tick values of the encoders
     *
     * @return Returns in this order: Vertical Right, Vertical Left, Horizontal
     */
    int[] returnRaw();

    /**
     * Sets the orientation of the odometry calibration system
     *
     * @param angle the angle to be at
     * @throws InterruptedException Throws if the thread is interrupted during execution
     */
    void setOrientation(double angle) throws InterruptedException;


    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     *
     * @throws InterruptedException Throws if the Thread is interrupted while waiting for lock. If is interrupted, values are not changed
     */
    void threadMain() throws InterruptedException;


    /**
     * Reverses the left encoder
     */
    void reverseLeftEncoder();

    /**
     * Reverses the right encoder
     */
    void reverseRightEncoder();

    /**
     * Reverses the Horizontal encoder
     */
    void reverseHorizontalEncoder();

}
