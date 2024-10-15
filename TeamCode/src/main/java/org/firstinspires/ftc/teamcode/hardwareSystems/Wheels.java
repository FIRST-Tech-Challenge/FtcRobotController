package org.firstinspires.ftc.teamcode.hardwareSystems;

import java.util.HashSet;

import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class Wheels {
    // A modifier for much power the wheels run with (0.0 - 1.0)
    protected double motorPower = 1.0;
    // The gear ratio between the motor and wheel
    private static final double WHEEL_GEAR_RATIO = -1.0;
    // The circumference of the wheel in inches
    private static final double WHEEL_CIRCUMFERENCE = -1.0;

    protected final HashSet<DcMotor> MOTORS;

    public Wheels() {
        MOTORS = new HashSet<>();
    }

    public double getMotorPower() {
        return motorPower;
    }

    public void setMotorPower(double motorPower) {
        this.motorPower = motorPower;
    }

    /**
     * Get all the DcMotors that are used by this wheels system.
     *
     * @return A set that contains every DcMotor included by this wheels system.
     */
    public HashSet<DcMotor> getMotors() {
        return MOTORS;
    }
    
    /**
     * Drive the wheels.
     * 
     * @param drivePower Forward power.
     *                   Positive is forward, negative is backward.
     * @param turn       Rotation power.
     *                   Positive is clockwise, negative is counterclockwise.
     */
    public abstract void drive(double drivePower, double turn);

    /**
     * Drive the wheels.
     * 
     * @param x    Sideways power.
     *             Positive is rightward, negative is leftward.
     * @param y    Forward power.
     *             Positive is forward, negative is backward.
     * @param turn Rotation power.
     *             Positive is clockwise, negative is counterclockwise.
     */
    public abstract void drive(double x, double y, double turn);

    /**
     * Drive the robot a certain distance forward.
     * 
     * @param distance The distance that the robot travels in inches.
     *                 Positive is forward, negative is backward.
     */
    public abstract void driveDistance(double distance);

    /**
     * Drive the robot a certain distance in two dimensions.
     * 
     * @param forwardDistance  The distance that the robot travels forward in
     *                         inches.
     *                         Positive is forward, negative is backward.
     * @param sidewaysDistance The distance that the robot travels sideways in inches.
     *                         Positive is rightward, negative is leftward.
     */
    public abstract void driveDistance(double forwardDistance, double sidewaysDistance);

    /**
     * Rotate the robot a certain number of degrees.
     *
     * @param degrees How many degrees the robot turns.
     *                Positive is clockwise, negative is counterclockwise.
     */
    public abstract void turn(double degrees);
}
