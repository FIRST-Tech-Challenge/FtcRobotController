// Flywheel.java
// This class controls the flywheel motor, including PIDF configuration and speed control.
// It provides methods to set and get the flywheel's speed, and to check if the flywheel is at the target speed.

package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.broombots.sweep.Classes.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Flywheel subsystem for controlling a shooter or similar mechanism.
 * Handles PIDF configuration and velocity control using a DcMotorEx.
 */
@Config
public class Flywheel implements Subsystem {
    // Timer for elapsed time tracking
    private double targetSpeed = 0.0;
    // Current speed of the flywheel (rotations per minute)
    private double currentSpeed = 0.0;
    // The flywheel motor (DcMotorEx)
    private DcMotorEx flywheel = null;
    private boolean disabled = false;
    // Acceptable error threshold for speed (rotations per minute)
    public static double accuracyThreshold = 1;

    /**
     * Constructs a Flywheel instance and configures the motor and PIDF coefficients.
     * @param hardwareMap HardwareMap for accessing hardware devices
     * @param p Proportional constant for PIDF
     * @param i Integral constant for PIDF
     * @param d Derivative constant for PIDF
     */
    public Flywheel(HardwareMap hardwareMap, double p, double i, double d, double f){

        // Initialize the flywheel motor
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMotorEnable();
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        // Set PIDF coefficients for velocity control
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p,i,d,f));
    }

    @Override
    public void update(){
        if(disabled){
            return;
        }
        flywheel.setVelocity(targetSpeed);
    }
    @Override
    public void setDisabled(boolean disabled){
        if(disabled){
            flywheel.setPower(0);
        }
        this.disabled=disabled;
    }

    public double getCurrentSpeed(){
        return currentSpeed;
    }

    /**
     * Sets the target speed for the flywheel and updates the motor velocity.
     * @param rotationsPerMinute Target speed in rotations per minute
     */
    public void setTargetSpeed(double rotationsPerMinute){
        //This is the convesation factor for angular rate so you can get it in rotations per minute
        targetSpeed = 28*rotationsPerMinute*60;
    }
    public boolean isAtSpeed(){
        return Math.abs(currentSpeed - targetSpeed)<accuracyThreshold;
    }
}