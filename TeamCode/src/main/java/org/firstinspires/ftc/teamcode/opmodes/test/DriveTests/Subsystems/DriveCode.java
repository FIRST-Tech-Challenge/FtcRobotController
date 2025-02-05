package org.firstinspires.ftc.teamcode.opmodes.test.DriveTests.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveCode {

    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private IMU imu;

    private static final double JOYSTICK_DEADZONE = 0.05; // Dead zone for joystick input
    private static final double DEFAULT_FIXED_POWER = 0.5; // Default power level for driving
    private static final double STRAFE_SCALE = 1.0; // Scale for strafing
    private static final double DRIVE_SCALE = 1.0; // Scale for forward/backward movement

    // Constructor for initializing motors and IMU
    public DriveCode(DcMotor frontLeftMotor, DcMotor backLeftMotor,
                     DcMotor frontRightMotor, DcMotor backRightMotor, IMU imu) {
        this.frontLeftMotor = frontLeftMotor;
        this.backLeftMotor = backLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backRightMotor = backRightMotor;
        this.imu = imu;

        // Configure motor directions
        this.frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motors to brake mode for precision
        this.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Drive method for handling gamepad input and moving the robot
    public void drive(Gamepad gamepad1, double fixedPower) {
        fixedPower = fixedPower > 0 ? fixedPower : DEFAULT_FIXED_POWER; // Use provided power or default

        // Set power levels based on joystick input
        double y = Math.abs(gamepad1.left_stick_y) > JOYSTICK_DEADZONE ? -fixedPower * Math.signum(gamepad1.left_stick_y) : 0;
        double x = Math.abs(gamepad1.left_stick_x) > JOYSTICK_DEADZONE ? fixedPower * Math.signum(gamepad1.left_stick_x) : 0;
        double rx = Math.abs(gamepad1.right_stick_x) > JOYSTICK_DEADZONE ? fixedPower * Math.signum(gamepad1.right_stick_x) : 0;

        // Reset the IMU yaw when the options button is pressed
        if (gamepad1.options) {
            imu.resetYaw();
        }

        // Get the robot's current heading from the IMU
        double botHeading;
        try {
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        } catch (Exception e) {
            botHeading = 0; // Default to 0 if IMU data is unavailable
            System.out.println("IMU data unavailable: Defaulting to 0 heading");
        }

        // Adjust movement for the robot's heading
        double rotX = (x * Math.cos(-botHeading) - y * Math.sin(-botHeading)) * STRAFE_SCALE;
        double rotY = (x * Math.sin(-botHeading) + y * Math.cos(-botHeading)) * DRIVE_SCALE;

        // Calculate motor power with strafing adjustment
        double frontLeftPower = rotY + rotX + rx;
        double backLeftPower = rotY - rotX + rx;
        double frontRightPower = rotY - rotX - rx;
        double backRightPower = rotY + rotX - rx;

        // Normalize motor power values for safety
        double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(
                Math.abs(backLeftPower), Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))
        ));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set motor powers
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        // Telemetry for debugging motor power values
        System.out.println("Front Left Power: " + frontLeftPower);
        System.out.println("Back Left Power: " + backLeftPower);
        System.out.println("Front Right Power: " + frontRightPower);
        System.out.println("Back Right Power: " + backRightPower);
    }
}
