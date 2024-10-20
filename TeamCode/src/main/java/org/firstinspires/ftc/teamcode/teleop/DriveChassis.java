package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

/**
 * A class with all of the common data about our robot chassis so we can reuse it in multiple OpModes without reusing too much code
 */
public class DriveChassis {
    // Initialize our motors
    protected DcMotorEx leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;

    // Info about our robots design
    protected final int TICKS_PER_REVOLUTION = 28;
    protected final double DRIVE_GEAR_RATIO = 20;
    protected final double WHEEL_CIRCUMFERENCE = 23.94; // In CM

    protected IMU imu;

    public DriveChassis(OpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;

        // Instantiate each motor
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "rightRear");

        // Instantiate the imu
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                ))
        );
        imu.resetYaw();
    }
}
