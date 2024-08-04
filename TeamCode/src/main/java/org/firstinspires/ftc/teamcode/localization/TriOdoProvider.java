package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.robotcore.hardware.DcMotor;

public interface TriOdoProvider {
    DcMotor getLeftEncoder();
    DcMotor getRightEncoder();
    DcMotor getCenterEncoder();

    /**
     * Number of encoder ticks per revolution of the encoder wheel.
     * @return ticks per revolution
     */
    default int getEncoderTicksPerRevolution() { return 8192; }

    /**
     * Radius of the encoder wheels.
     * @return Radius in inches
     */
    default double getEncoderWheelRadius() { return 0.69; }

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
