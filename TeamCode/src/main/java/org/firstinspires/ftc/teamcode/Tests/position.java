package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Position Tracking with IMU and Gamepad", group = "Linear Opmode")
public class position extends LinearOpMode {

    private DcMotorEx frontLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx backRightMotor;
    private BHI260IMU imu;

    private static final double WHEEL_DIAMETER_MM = 104; // Diameter of the wheel in mm
    private static final double TICKS_PER_REV = 537.7; // Encoder ticks per revolution for GoBilda 312 RPM motor
    private static final double MM_PER_TICK = (Math.PI * WHEEL_DIAMETER_MM) / TICKS_PER_REV;

    private double xPosition = 0; // X position in mm
    private double yPosition = 0; // Y position in mm
    private double heading = 0;  // Heading in radians

    @Override
    public void runOpMode() {
        // Initialize hardware
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "1");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "2");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "0");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "3");
        imu = hardwareMap.get(BHI260IMU.class, "imu");

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu.initialize(
                new com.qualcomm.robotcore.hardware.IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)
                )
        );

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        int lastFLPosition = 0;
        int lastFRPosition = 0;
        int lastBLPosition = 0;
        int lastBRPosition = 0;

        while (opModeIsActive()) {
            // Basic movement using gamepad
            double drive = -gamepad1.left_stick_y; // Forward and backward
            double strafe = gamepad1.left_stick_x; // Left and right
            double rotate = gamepad1.right_stick_x; // Rotation

            // Calculate motor powers
            double frontLeftPower = drive + strafe + rotate;
            double frontRightPower = drive - strafe - rotate;
            double backLeftPower = drive - strafe + rotate;
            double backRightPower = drive + strafe - rotate;

            // Normalize motor powers if any exceed 1.0
            double maxPower = Math.max(1.0, Math.abs(frontLeftPower));
            maxPower = Math.max(maxPower, Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backLeftPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));

            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;

            // Set motor powers
            frontLeftMotor.setPower(frontLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backLeftMotor.setPower(backLeftPower);
            backRightMotor.setPower(backRightPower);

            // Update position and heading
            int currentFLPosition = frontLeftMotor.getCurrentPosition();
            int currentFRPosition = frontRightMotor.getCurrentPosition();
            int currentBLPosition = backLeftMotor.getCurrentPosition();
            int currentBRPosition = backRightMotor.getCurrentPosition();

            int deltaFL = currentFLPosition - lastFLPosition;
            int deltaFR = currentFRPosition - lastFRPosition;
            int deltaBL = currentBLPosition - lastBLPosition;
            int deltaBR = currentBRPosition - lastBRPosition;

            lastFLPosition = currentFLPosition;
            lastFRPosition = currentFRPosition;
            lastBLPosition = currentBLPosition;
            lastBRPosition = currentBRPosition;

            double deltaX = MM_PER_TICK * (deltaFL + deltaFR + deltaBL + deltaBR) / 4.0;
            double deltaY = MM_PER_TICK * (-deltaFL + deltaFR + deltaBL - deltaBR) / 4.0;

            heading = Math.toRadians(imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate);

            xPosition += deltaX * Math.cos(heading) - deltaY * Math.sin(heading);
            yPosition += deltaX * Math.sin(heading) + deltaY * Math.cos(heading);

            telemetry.addData("X Position (mm):", xPosition);
            telemetry.addData("Y Position (mm):", yPosition);
            telemetry.addData("Heading (rad):", heading);
            telemetry.update();
        }
    }
}
