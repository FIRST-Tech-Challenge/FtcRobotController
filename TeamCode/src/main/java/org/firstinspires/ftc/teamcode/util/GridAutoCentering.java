package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumSubsystem;

//This util would turn the drivebase 90 degrees exactly or can maintain your angle if you bumped into something
public class GridAutoCentering {
    // Reference to the Mecanum drive subsystem for controlling robot movement
    private MecanumSubsystem mecanumSubsystem;

    // Reference to the GyroOdometry class to get the robot's heading
    private GyroOdometry gyroOdometry;

    // PID controller constants
    private double Kp = -1.5; // Proportional gain
    private double Kd = 0;    // Derivative gain
    private double Ki = 0;    // Integral gain
    private double Ff = 0;    // Feed-forward term

    // Sum of all past errors (for integral term)
    private double integralSum = 0;

    // Timer to measure time between PID updates
    private ElapsedTime timer;

    // Stores the previous loop's error (for derivative calculation)
    private double lastError = 0;

    // The base reference angle (e.g., orientation when reset is called)
    public double baseAngle = 0;

    // The current PID error (difference between target and actual angle)
    public double error;

    // Rate of change of error (used for derivative term)
    public double derivative;

    // Output value of the PID controller (for debugging or plotting)
    private double graphRotationalVel = 0;

    // The desired angle the robot should maintain
    private double targetAngle = 0;

    // Constructor initializes subsystems and timer
    public GridAutoCentering(MecanumSubsystem mecanumSubsystem, GyroOdometry gyroOdometry) {
        this.mecanumSubsystem = mecanumSubsystem;
        this.gyroOdometry = gyroOdometry;
        timer = new ElapsedTime(); // Start a new timer instance
    }

    // Resets the base angle to the current heading and aligns the target to it
    public void reset(){
        baseAngle = gyroOdometry.getAngle();
        targetAngle = baseAngle;
    }

    // Resets both base and target angle to zero (absolute reset)
    public void resetZero(){
        baseAngle = 0;
        targetAngle = baseAngle;
    }

    // Allows tuning of PIDF constants from outside the class
    public void setConstants(double kp, double ki, double kd, double ff){
        Kp = kp;
        Ki = ki;
        Kd = kd;
        Ff = ff;
    }

    // Adds an offset to the target angle relative to base angle
    public void offsetTargetAngle(double angle){
        targetAngle = baseAngle + angle;
    }

    // Main PID loop that runs when 'run' is true
    public void process(boolean run){
        if(run) {
            timer.reset(); // Reset timer to measure this cycle duration

            // Calculate error between current angle and desired target angle
            error = targetAngle - gyroOdometry.getAngle();

            // Wrap-around correction to keep angle between -π and π
            if (error >= Math.PI) {
                error -= Math.PI * 2;
            } else if (error <= -Math.PI) {
                error += Math.PI * 2;
            }

            // Accumulate integral term
            integralSum += error * timer.time();

            // Compute derivative of error
            derivative = (error - lastError) / timer.time();

            // Store current error for next iteration
            lastError = error;

            // Compute PID output (excluding integral for now)
            graphRotationalVel = Ff + (error * Kp) + (derivative * Kd);

            // Apply rotational adjustment if error is significant
            if (Math.abs(error) > 0.0025) {
                mecanumSubsystem.partialMoveAdjustment(true, 0.0, 0.0, graphRotationalVel /*+(integralSum * Ki)*/);
            }
        } else {
            // Stop any rotational movement if not running
            mecanumSubsystem.partialMoveAdjustment(true, 0.0, 0.0, 0.0);
        }
    }

    // Getter for the last computed PID output (for telemetry/debugging)
    public double getGraphRotationalVel(){
        return graphRotationalVel;
    }

    // Alternate process method that skips issuing drive commands (used in diagnostics or secondary logic)
    public void secondaryProcess(boolean run){
        timer.reset(); // Reset timer

        // Calculate angle error and wrap it into valid range
        error = targetAngle - gyroOdometry.getAngle();
        if (error >= Math.PI){
            error -= Math.PI*2;
        } else if (error <= -Math.PI){
            error += Math.PI*2;
        }

        // Compute integral and derivative terms
        integralSum += error * timer.time();
        derivative = (error - lastError) / timer.time();
        lastError = error;

        // Placeholder for future control logic (currently commented out)
        if (Math.abs(error) > 0.005 && run){
            // mecanumSubsystem.fieldOrientedMove(0, 0, (error * Kp) + (derivative * Kd)/* + (integralSum * Ki)*/, 0);
        }
    }

    // Returns the current target angle
    public double getTargetAngle() {
        return targetAngle;
    }

    // Sets the target angle to a new value
    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }
}
