package org.firstinspires.ftc.teamcode.hardwareSystems;

import java.util.HashSet;

import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class Wheels {
    protected final HashSet<DcMotor> MOTORS;
    protected final MotorType MOTOR_TYPE;

    // A modifier for much power the wheels run with (0.0 - 1.0)
    protected double motorPower = 1.0;
    protected final double TICKS_PER_INCH;

    public Wheels(HashSet<DcMotor> motors) {
        this(motors, MotorType.TETRIX_TORQUENADO, 100);
    }

    /**
     * Instantiate the a wheels object.
     *
     * @param motors       All the motors used by the robot.
     * @param motorType    The motor type used by the robot.
     *                     Assumes that all motors in the System are the same
     *                     (THEY SHOULD BE!)
     * @param ticksPerInch The number of ticks needed to move the robot by one inh.
     */
    public Wheels(HashSet<DcMotor> motors, MotorType motorType, double ticksPerInch) {
        this.MOTORS = motors;
        this.MOTOR_TYPE = motorType;

        this.TICKS_PER_INCH = ticksPerInch;
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
