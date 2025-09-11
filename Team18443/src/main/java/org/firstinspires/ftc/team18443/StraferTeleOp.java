package org.firstinspires.ftc.teamm18443;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;

// ****************************************************************************
//  StraferTeleOp.java                                       GalacticLions2526
// ****************************************************************************
//   Description:
//      This OpMode executes a field-centric Teleop for a mecanum drive robot
//      The code is structured as a LinearOpMode
//
//   Usage:
//      - Deploy as TeleOp via FTC Driver Station 
//
// ****************************************************************************
// This program is released under the BSD-3-Clause-Clear License
// See LICENSE file in root of this repository
// ****************************************************************************

@TeleOp(name="Strafer Tele Op", group="TeleOp")
public class StraferTeleOp extends LinearOpMode {
    
    @Override
    public void runOpMode() {

// ----------------------------------------------------------------------------
//    Define and Initialize the Motors
// ----------------------------------------------------------------------------
// Note: Make sure the ID's match in your configuration
        
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "lf");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "rf");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "lb");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "rb");

        // Reverse one side of the motors for mecanum drive to ensure consistent forward movement
        // If the robot drives backwards, reverse the other side instead
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

// ----------------------------------------------------------------------------
//    Define and Initialize the REV Hub's IMU (Inertial measurement unit)
// ----------------------------------------------------------------------------
// Note: Adjust orientation parameters below to match your robot's mounting
        
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // By default, IMU assumes REV Hub is mounted with logo up and USB port facing forward
        imu.initialize(parameters);
        imu.resetYaw();


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses START)
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

// ----------------------------------------------------------------------------
//    Primary Driver Controls (gamepad1)
// ----------------------------------------------------------------------------
            
            double y = -gamepad1.left_stick_y; // Forward/backward (negative because up is negative)
            double x = gamepad1.left_stick_x * 1.1; // Strafe left/right (scaled to compensate for imperfect strafing)
            double rx = gamepad1.right_stick_x; // Rotation
            
            // Convert field-relative joystick inputs to robot-relative using IMU heading
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double X = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double Y = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Normalize motor powers to keep within [-1, 1] while maintaining power ratios
            double denominator = Math.max(Math.abs(Y) + Math.abs(X) + Math.abs(rx), 1);
            double frontLeftPower = (Y + X + rx) / denominator;
            double backLeftPower = (Y - X + rx) / denominator;
            double frontRightPower = (Y - X - rx) / denominator;
            double backRightPower = (Y + X - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);
            
            // Reset IMU yaw angle to zero manually by pressing the 'guide' button
            if (gamepad1.guide) {
                imu.resetYaw();
            }

// ----------------------------------------------------------------------------
//    Secondary Driver Controls (gamepad2)
// ----------------------------------------------------------------------------
        }
    }
}
