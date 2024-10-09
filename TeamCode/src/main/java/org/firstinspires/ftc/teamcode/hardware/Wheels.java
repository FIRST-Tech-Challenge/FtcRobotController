package org.firstinspires.ftc.teamcode.hardware;

import java.util.HashSet;

import com.qualcomm.robotcore.hardware.*;

public abstract class Wheels {
    protected HashSet<DcMotor> motors;

    // A modifier for much power the wheels run with (0.0 - 1.0)
    protected double motorPower = 1.0;

    public Wheels() {
        motors = new HashSet<>();
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
        return motors;
    }

    /**
     * Drive the wheels.
     * 
     * @param drive Forward input
     * @param turn  Turn input
     */
    public abstract void drive(double drivePower, double turn);

    /**
     * Drive the wheels.
     * 
     * @param x    Sideways input
     * @param y    Forward input
     * @param turn Rotation input
     */
    public abstract void drive(double x, double y, double turn);

    /**
     * Drive the robot a certain distance forward.
     * 
     * @param distance The distance that the robot travels in inches.
     *                 Positive is forward and negative is backward.
     */
    public abstract void driveDistance(double distance);

    /**
     * Drive the robot a certain distance in two dimensions.
     * 
     * @param forwardDistance  The distance that the robot travels forward in
     *                         inches.
     *                         Positive is forward and negative is backward.
     * @param sidewaysDistance The distance that the robot travels sidways in inches.
     *                         Negative is leftward and positive is righward.
     */
    public abstract void driveDistance(double forwardDistance, double sidewaysDistance);
}
