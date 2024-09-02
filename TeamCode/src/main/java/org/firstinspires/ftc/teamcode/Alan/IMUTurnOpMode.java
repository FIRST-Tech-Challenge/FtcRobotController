package org.firstinspires.ftc.teamcode.Alan;


import android.graphics.drawable.GradientDrawable;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Disabled
public class IMUTurnOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        IMU imu;
        imu = hardwareMap.get(IMU.class, "imu");

        boolean success = imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                         )
                )
        );
        if (!success) {
            Log.d("alan", "imu initialization failed");
            return;

        }
        imu.resetYaw();
        Log.d("alan", "reset yaw done");
        YawPitchRollAngles robotOrientation;
        AngularVelocity robotAcceleration;
        waitForStart();
        while (opModeIsActive()) {
            robotOrientation = imu.getRobotYawPitchRollAngles();
            robotAcceleration = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            double zAccel = robotAcceleration.zRotationRate;
            double yaw = robotOrientation.getYaw(AngleUnit.DEGREES);

            telemetry.addData("Yaw:", yaw);
            telemetry.addData("------------------------------", "");
            telemetry.addData("Acceleration on Z axis: ", "" + zAccel);
            telemetry.update();
        }
    }
}