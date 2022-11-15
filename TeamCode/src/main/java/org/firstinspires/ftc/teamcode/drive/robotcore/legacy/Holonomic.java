package org.firstinspires.ftc.teamcode.drive.robotcore.legacy;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.library.functions.Stoppable;

public interface Holonomic extends Stoppable {
    /**
     * Sets runMode for all motors to {@link DcMotor.RunMode#RUN_WITHOUT_ENCODER}
     *
     * @see Holonomic#runWithoutEncoderVectored(double, double, double, double) for mathematically correct movement
     *
     * @param x left-to-right direction
     * @param y back-to-front direction
     * @param z rotation power
     */
    void runWithoutEncoder(double x, double y, double z);

    /**
     * Converts proportional constants in cartesian directions to 45-degree translated powers for optimal driving
     * Sets motor powers and runMode for all motors to {@link DcMotor.RunMode#RUN_WITHOUT_ENCODER}
     * @param x proportional constant representing 45-degree translation of x-axis
     *               (runs from back left to front right)
     * @param y proportional constant representing 45-degree translation of y-axis
     *               (runs from back right to front left)
     * @param z value added to motor powers to create rotation
     * @param offsetTheta increases or decreases angle translation
     *                    (could be used for field-oriented drive with IMU)
     */
    void runWithoutEncoderVectored(double x, double y, double z, double offsetTheta);

    /**
     * Uses 45-degree translations of the cartesian system to set motor powers
     * Sets runMode for all motors to {@link DcMotor.RunMode#RUN_WITHOUT_ENCODER}
     * @param xPrime proportional constant representing 45-degree translation of x-axis
     *               (runs from back left to front right)
     * @param yPrime proportional constant representing 45-degree translation of y-axis
     *               (runs from back right to front left)
     * @param z value added to motor powers to create rotation
     */
    void runWithoutEncoderPrime(double xPrime, double yPrime, double z);

    /**
     * Uses cartesian directions to set encoder targets and motor power for strafing
     * @param xTarget left-to-right distance (inches) for the robot to travel
     * @param yTarget back-to-front distance (inches) for the robot to travel
     * @param inputPower maximum power that motors will operate at
     */
    void runUsingEncoder(double xTarget, double yTarget, double inputPower);

    /**
     * Turns the robot using encoder measurements based on a degree parameter
     * @param degrees angle in degrees for the robot to turn
     *                positive angle = clockwise
     *                negative angle = counter-clockwise
     * @param power the maximum power that motors will operate at
     */
    void turnUsingEncoder(double degrees, double power);


    /**
     * Provides information as to whether motors are still attempting to reach encoder targets
     * @return true if all four motors are busy
     */
    boolean motorsAreBusy();

    /**
     * Sets all drivetrain motors to the same DcMotor.RunMode
     * @param runMode the RunMode to be set
     */
    void setMotorsMode(DcMotor.RunMode runMode);

    /**
     * Sets zero power behavior for all drivetrain motors
     * @param zeroPowerBehavior the DcMotor.ZeroPowerBehavior to be set
     */
    void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior);
}