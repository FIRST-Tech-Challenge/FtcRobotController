package org.firstinspires.ftc.teamcode.Grande;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class IMUExample extends LinearOpMode {
    // IMU website explanation: https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html
    private IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu"); //chub i2c port 0

        IMU.Parameters parameters;

        parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(parameters);

        waitForStart();

        while(opModeIsActive()){
            YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();

            double yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
            double pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
            double roll  = robotOrientation.getRoll(AngleUnit.DEGREES);

            telemetry.addData("yaw", yaw);
            telemetry.addData("pitch", pitch);
            telemetry.addData("roll", roll);
            telemetry.update();
        }

    }
}
