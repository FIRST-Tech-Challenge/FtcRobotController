package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Teleop")
public class Teleop extends OpMode {
    private DcMotorEx leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;
    private final double DRIVE_GEAR_RATIO = 20;
    private double drivePower = 1.0;

    private IMU imu;
    private YawPitchRollAngles orientation;
    private double yaw;

    @Override
    public void init() {
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

        orientation = imu.getRobotYawPitchRollAngles();
    }

    @Override
    public void loop() {
        orientation = imu.getRobotYawPitchRollAngles();
        yaw = orientation.getYaw();

        float moveXInput = gamepad1.left_stick_x;
        float moveYInput = -gamepad1.left_stick_y;
        float rotationInput = gamepad1.right_stick_x;


    }
}