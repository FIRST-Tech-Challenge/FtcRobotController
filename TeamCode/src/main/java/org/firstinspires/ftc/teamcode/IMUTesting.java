package org.firstinspires.ftc.teamcode;

import org.inventors.ftc.robotbase.hardware.RevIMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "IMUTesting", group = "Tests")
@Disabled
public class IMUTesting extends LinearOpMode {
    RevIMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = new RevIMU(hardwareMap, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);

        imu.init(parameters);

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Rotation 0: ", imu.getYawPitchRoll()[0]);
            telemetry.addData("Rotation 1: ", imu.getYawPitchRoll()[1]);
            telemetry.addData("Rotation 2: ", imu.getYawPitchRoll()[2]);
            telemetry.update();
        }
    }
}
