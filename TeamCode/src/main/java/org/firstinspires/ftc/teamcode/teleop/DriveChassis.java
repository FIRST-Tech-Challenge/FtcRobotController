package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class DriveChassis {
    protected DcMotorEx leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;
    protected final int TICKS_PER_REVOLUTION = 28;
    protected final double DRIVE_GEAR_RATIO = 60;
    protected final double WHEEL_CIRCUMFERENCE = 23.94; // In CM

    protected IMU imu;

    public DriveChassis(OpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;

        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "rightRear");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                ))
        );
        imu.resetYaw();
    }
}
