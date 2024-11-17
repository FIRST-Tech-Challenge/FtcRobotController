package org.firstinspires.ftc.teamcode.mmooover;

import org.firstinspires.ftc.teamcode.hardware.Encoder;

// An interface describes what methods a class needs to have
// The hardware.java has methods and other stuff that can be used no matter what robot hub thing
// so we used the interface which describes what methods we will provide for this specific program
public interface TriOdoProvider {
    Encoder getLeftEncoder();
    Encoder getRightEncoder();
    Encoder getCenterEncoder();

    /**
     * Number of encoder ticks per revolution of the encoder wheel.
     * @return ticks per revolution
     */
    default int getEncoderTicksPerRevolution() { return 8192; }

    /**
     * Radius of the encoder wheels.
     * @return Radius in inches
     */
    default double getEncoderWheelRadius() { return 0.70; }

    /**
     * Distance between the two parallel encoders.
     * @return Distance between encoders in inches
     */
    double getTrackWidth();

    /**
     * Distance from the center of the robot to the center of the forward encoder.
     * May be negative if the 'forward' encoder is behind the center of the robot.
     * @return Distance in inches. Positive is forward.
     */
    double getForwardOffset();
}
