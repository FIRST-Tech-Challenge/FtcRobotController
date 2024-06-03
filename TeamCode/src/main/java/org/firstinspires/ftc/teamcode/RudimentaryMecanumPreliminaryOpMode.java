package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hadrware.DcMotorSimple;

/*
 * Made by Peark Kamalu on May 31 2024. 
 * 
 * This is a basic op mode for controling mecanum wheels
 * that is commonly used by many teams for controling their drive chain
 * in a common configuration such as this one.
 *  
 * This is based on https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
 * Made for one controller. 
 * 
 * 
 */

@TeleOp

public class RudimentaryMecanumPreliminaryOpMode extends LinearOpMode {
    // Motor Primatives
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    @Override
    public void runOpMode() {
        // Setup Motors
        // Need to set these up on the robot
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        // Reverse the motors and see if they work
        frontRightMotor.setDirecton(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirecton(DcMotorSimple.Direction.REVERSE);

        // Telemetry
        telemetry.addData("Status", "Initalized");
        telemetry.update();


        waitForStart();
        

        // Variables
        // Left stick is used for controling planar motion
        // Right stick is used for turn, but only using x axis to get variable speed

        // Strafing is slower than forwards b/c of lost friction, reqiring more power and thus scaling
        // Adjust this value for driver's preverence
        double strafeScale = 1.2; 

        double forwardPower = -gamepad1.left_stick_y; // Y is reversed
        double strafePower = gamepad1.left_stick_x * strafeScale; // Strafe = sideways power.

        double turnPower = gamepad1.right_stick_x; 

        // SDK clips power at 1, so we need to scale everything else down
        // Divde everything by largest number, so largest number will be one
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        // See gm0's page for mecanum wheels for why.
            // Make `forwardPower` negative if this is the fact.
            // for troubleshooting, comment out everything after forward power and see if they go in correct direction
        double frontLeftMotorPower = (forwardPower + strafePower + turnPower) / denominator; 
        double frontRightMotorPower = (forwardPower - strafePower - turnPower) / denominator;
        double backLeftMotorPower = (forwardPower - strafePower - turnPower) / denominator;     
        double backRightMotorPower =  (forwardPower + strafePower - turnPower) / denominator;
        
        while (opModeIsActive()) {
            // See gm0's page for mecanum wheels for why.
            // Make `forwardPower` negative if this is the fact.
            // for troubleshooting, comment out everything after forward power and see if they go in correct direction
            frontLeftMotor.setPower(frontLeftMotorPower);
            frontRightMotor.setPower(frontRightMotorPower);
            backLeftMotor.setPower(backLeftMotorPower);
            backRightMotor.setPower(backRightMotorPower);

        }
    }
}
