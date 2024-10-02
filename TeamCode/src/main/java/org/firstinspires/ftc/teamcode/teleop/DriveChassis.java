package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class DriveChassis {
    protected DcMotorEx leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;
    protected final double DRIVE_GEAR_RATIO = 60;

    protected IMU imu;

    public DriveChassis(OpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;

        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "rightBack");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                ))
        );
    }
}
