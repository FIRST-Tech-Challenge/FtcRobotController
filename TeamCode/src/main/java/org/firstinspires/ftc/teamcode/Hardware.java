package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware extends LinearOpMode {


    // Good Luck!
    //You should put constants here

    protected DcMotor frontLeft, frontRight, backLeft, backRight, clawStrafe, clawRotate ;
    protected Servo clawGrab;
    // Setup your drivetrain (Define your motors etc.)
    public void hardwareSetup() {

        // Define your methods of driving (Encoder drive, tank drive, etc.

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        clawStrafe = hardwareMap.dcMotor.get("clawStrafe");
        clawRotate = hardwareMap.dcMotor.get("clawRotate");
        clawGrab = hardwareMap.servo.get("clawGrab");

        telemetry.addData("Status:","Setup Complete");
        telemetry.update();
    }
    // Pinchas should make an encoder drive

    // Last thing is an empty runOpMode because it's a linearopmode
    @Override
    public void runOpMode() throws InterruptedException {

    }
}
