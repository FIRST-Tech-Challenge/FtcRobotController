package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="Field Oriented Drive", group="TeleOp")
public class FieldCentricDrive extends OpMode {

    // Motor declarations
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // IMU Sensor
    private IMU imu;
    private double headingOffset = 0.0; // To reset field orientation

    @Override
    public void init() {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        // Reverse right-side motors
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class,"imu");
        IMU.Parameters myIMUparameters;
        myIMUparameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));


        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Get IMU heading (yaw)
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double currentHeading = -angles.getYaw(AngleUnit.RADIANS) - headingOffset;

        // Get gamepad inputs
        double y = -gamepad1.left_stick_y; // Forward/Backward
        double x = gamepad1.left_stick_x;  // Strafe Left/Right
        double turn = gamepad1.right_stick_x; // Rotation

        // Field-oriented transformation
        double cosA = Math.cos(currentHeading);
        double sinA = Math.sin(currentHeading);
        double fieldX = x * cosA - y * sinA;
        double fieldY = x * sinA + y * cosA;

        // Mecanum drive calculations
        double frontLeftPower = fieldY + fieldX + turn;
        double frontRightPower = fieldY - fieldX - turn;
        double backLeftPower = fieldY - fieldX + turn;
        double backRightPower = fieldY + fieldX - turn;

        // Normalize motor power
        double max = Math.max(1.0, Math.abs(frontLeftPower));
        max = Math.max(max, Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        frontLeft.setPower(frontLeftPower / max);
        frontRight.setPower(frontRightPower / max);
        backLeft.setPower(backLeftPower / max);
        backRight.setPower(backRightPower / max);

        if (gamepad1.a) resetHeading();

        // Telemetry output
        telemetry.addData("Heading (degrees)", Math.toDegrees(currentHeading));
        telemetry.addData("Field X", fieldX);
        telemetry.addData("Field Y", fieldY);
        telemetry.update();
    }

    // Reset the heading offset (bind this to a button if needed)
    public void resetHeading() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        headingOffset = angles.getYaw(AngleUnit.RADIANS);
    }
}