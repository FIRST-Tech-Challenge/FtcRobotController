package org.firstinspires.ftc.teamcode.opmodes.test.DriveTests;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Robot Control", group="TeleOp")
@Disabled
public class TeleOpTest2 extends OpMode {

    // Define motors for drivetrain and arm components
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor linearSlideMotor;
    private DcMotor ArmMotor;

    // Power settings
    private static final double DRIVE_POWER = 0.8;   // Drivetrain power
    private static final double SLIDE_POWER = 0.8;  // Linear slide power
    private static final double ARM_POWER = 0.5;    // Arm control power

    @Override
    public void init() {
        // Initialize drivetrain motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        // Initialize arm components
        linearSlideMotor = hardwareMap.get(DcMotor.class, "linearSlideMotor");
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");

        // Set motor directions (adjust based on your setup)
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        linearSlideMotor.setDirection(DcMotor.Direction.REVERSE);
        ArmMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up the ArmMotor's encoder position
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setTargetPosition(0);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        // Drivetrain control with gamepad joysticks
        double drive = -gamepad1.left_stick_x;  // Forward and backward
        double strafe = gamepad1.left_stick_y; // Left and right
        double turn = gamepad1.right_stick_x;  // Rotation

        // Calculate power for each motor
        double frontLeftPower = drive + strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backLeftPower = drive - strafe + turn;
        double backRightPower = drive + strafe - turn;

        // Normalize motor powers if any exceeds the range [-1, 1]
        double maxPower = Math.max(1.0, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        frontLeftPower /= maxPower;
        frontRightPower /= maxPower;
        backLeftPower /= maxPower;
        backRightPower /= maxPower;

        // Set power to drivetrain motors
        frontLeftMotor.setPower(frontLeftPower * DRIVE_POWER);
        frontRightMotor.setPower(frontRightPower * DRIVE_POWER);
        backLeftMotor.setPower(backLeftPower * DRIVE_POWER);
        backRightMotor.setPower(backRightPower * DRIVE_POWER);

        // Linear slide control
        if (gamepad1.dpad_up) {
            ArmMotor.setPower(SLIDE_POWER); // Move up
        } else if (gamepad1.dpad_down) {
            ArmMotor.setPower(-SLIDE_POWER); // Move down
            ;
        } else {
            ArmMotor.setPower(0); // Stop
        }

        // ArmMotor control with positional reset
         if (gamepad1.dpad_left) {
            linearSlideMotor.setPower(SLIDE_POWER); // Move up
             telemetry.addData("dpad left", gamepad1.dpad_left);

         } else if (gamepad1.dpad_right) {
            linearSlideMotor.setPower(-SLIDE_POWER); // Move down
             telemetry.addData("dpad right", gamepad1.dpad_right);

         } else {
            linearSlideMotor.setPower(0); // Stop
        }

        // Telemetry for debugging
        telemetry.addData("dpad up", gamepad1.dpad_up);
        telemetry.addData("dpad down", gamepad1.dpad_down);
        telemetry.addData("Front Left Power", frontLeftPower);
        telemetry.addData("Front Right Power", frontRightPower);
        telemetry.addData("Back Left Power", backLeftPower);
        telemetry.addData("Back Right Power", backRightPower);
        telemetry.addData("Linear Slide Power", linearSlideMotor.getPower());
        telemetry.addData("Arm Power", ArmMotor.getPower());
        telemetry.addData("Arm Position", ArmMotor.getCurrentPosition());

        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop all motors when OpMode ends
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        linearSlideMotor.setPower(0);
        ArmMotor.setPower(0);
    }
}
