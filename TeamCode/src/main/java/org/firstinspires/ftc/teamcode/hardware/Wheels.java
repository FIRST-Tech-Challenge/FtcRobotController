package org.firstinspires.ftc.teamcode.hardware;

import java.util.HashSet;

public abstract class Wheels {
    // A modifier for much power the wheels run with (0.0 - 1.0)
    protected double wheelPower = 1.0;
    
    public Wheels() {

    }

    /**
     * Get all the DcMotors that are used by this wheels system.
     * @return A set that contains every DcMotor included by this wheels system.
     */
    public abstract HashSet<DcMotor> getAllMotors();

    /**
     * Drive the wheels.
     * 
     * @param drive Forward input
     * @param turn Turn input
     */
    public abstract void drive(double drivePower, double y);

    /**
     * Drive the wheels.
     * 
     * @param x Sideways input
     * @param y Forward input
     * @param rotate Rotation input
     */
    public abstract void drive(double x, double y, double rotate);
}
