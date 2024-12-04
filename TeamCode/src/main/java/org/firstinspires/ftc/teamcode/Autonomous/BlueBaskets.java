package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Utils.ChassisDriver;

@Autonomous (name = "BlueBaskets")
public class BlueBaskets extends LinearOpMode {
    DcMotorEx lb, lf, rb, rf;
    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        lb = hardwareMap.get(DcMotorEx.class, "lb");
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rb = hardwareMap.get(DcMotorEx.class, "rb");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        ChassisDriver.initializeMotors(lf, rf, lb, rb);
        ChassisDriver.resetWheelEncoders(lf, rf, lb, rb);

        waitForStart();

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
    }
}
