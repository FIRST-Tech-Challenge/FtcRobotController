package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Chassis {
    public DcMotor fl, fr, bl, br;
    public IMU imu;

    public Chassis(HardwareMap hardwareMap) {
        // Initialize motors
        fl = hardwareMap.get(DcMotor.class, "frontLeft");
        fr = hardwareMap.get(DcMotor.class, "frontRight");
        bl = hardwareMap.get(DcMotor.class, "backLeft");
        br = hardwareMap.get(DcMotor.class, "backRight");

        // Set motor directions and zero power behavior
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");

        // Set up IMU parameters
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);
    }

    // Method to reset the robot's yaw angle
    public void resetYaw() {
        imu.resetYaw();
    }

    // Drive method with field-centric control
    public void drive(double x, double y, double rx) {
        // Get the robot's current heading (heading = where the robot is faced)
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Adjust the input values for field-centric control
        double adjustedX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double adjustedY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Optional adjustment to counteract imperfect strafing
        adjustedX = adjustedX * 1.1;

        // Calculate the motor powers
        double denominator = Math.max(Math.abs(adjustedY) + Math.abs(adjustedX) + Math.abs(rx), 1);
        double frontLeftPower = (adjustedY + adjustedX + rx) / denominator;
        double backLeftPower = (adjustedY - adjustedX + rx) / denominator;
        double frontRightPower = (adjustedY - adjustedX - rx) / denominator;
        double backRightPower = (adjustedY + adjustedX - rx) / denominator;

        // Set motor powers
        fl.setPower(frontLeftPower);
        fr.setPower(frontRightPower);
        bl.setPower(backLeftPower);
        br.setPower(backRightPower);
    }
}