package org.firstinspires.ftc.teamcode.Systems.Mechaisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class IntakeSlides {

    private DcMotorEx intakeSlideMotor;

    private double spoolDiam = 3.0; // Spool Diameter in cm
    private double extensionLimit = 60; // Extension Limit in cm

    private double ticksToCm = (Math.PI * spoolDiam) / (145.1); // Multiply ticks by this number to get distance in cm
    private double cmToTicks = 1 / ticksToCm; // Multiply cm by this number to get distance in encoder ticks

    private double targetCM = 0;

    public IntakeSlides (DcMotorEx motor) {
        intakeSlideMotor = motor;
    }

    public void command() {

    }

    public void setTarget(double cm) {
        targetCM = cm;
    }

}
