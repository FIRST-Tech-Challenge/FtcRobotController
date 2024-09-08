package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoMain")
public class Auto extends LinearOpMode{
//    static final double TICKS_PER_MOTOR_REV = ;
//    static final double DRIVE_GEAR_REDUCTION = 1.0;
//    static final double WHEEL_DIAMETER_INCHES = ;
//    final private double DRIVE_SPEED = 0.6;
//    static final double TICKS_PER_INCH = (TICKS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    final private double DEFAULT_POWER = 0.95;
    Hardware hw = new Hardware(this);

    @Override
    public void runOpMode() throws InterruptedException {
        hw.init(hardwareMap);
    }

    public void drive(double distance) {

    }

    public void drive(double distance, double power) {

    }

    public void rotate(double degrees) {

    }

    public void rotate(double degrees, double power) {

    }
}
