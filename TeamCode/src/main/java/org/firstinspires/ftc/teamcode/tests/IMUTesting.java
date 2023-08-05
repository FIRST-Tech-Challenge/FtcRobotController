package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Disabled
@TeleOp(name="IMU Test", group="Tests")
public class IMUTesting extends LinearOpMode {
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbFacingDirection);

    IMU imu;

    YawPitchRollAngles orientation;

    double yaw, pitch, roll;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        waitForStart();

        while(opModeIsActive()) {
            orientation = imu.getRobotYawPitchRollAngles();

            yaw = orientation.getYaw(AngleUnit.DEGREES);
            pitch = orientation.getPitch(AngleUnit.DEGREES);
            roll = orientation.getRoll(AngleUnit.DEGREES);

            telemetry.addData("Yaw: ", yaw);
            telemetry.addData("Pitch: ", pitch);
            telemetry.addData("Roll: ", roll);
            telemetry.update();
        }
    }
}