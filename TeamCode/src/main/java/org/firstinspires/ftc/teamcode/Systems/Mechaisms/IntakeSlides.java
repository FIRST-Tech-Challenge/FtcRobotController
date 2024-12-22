package org.firstinspires.ftc.teamcode.Systems.Mechaisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class IntakeSlides {

    private DcMotorEx intakeSlideMotor;

    private double spoolDiam = 3.0; // Spool Diameter in cm
    private double extensionLimit = 60; // Extension Limit in cm

    private double ticksToCm = (Math.PI * spoolDiam) / (145.1); // Multiply ticks by this number to get distance in cm
    private double cmToTicks = 1 / ticksToCm; // Multiply cm by this number to get distance in encoder ticks

    private double targetCM = 0;

    public IntakeSlides (Hardware hardware) {
        intakeSlideMotor = hardware.intakeSlideMotor;
    }

    public void command() {

    }

    public void setTarget(double cm) {
        targetCM = cm;
    }

    public void update() {
    }
}
