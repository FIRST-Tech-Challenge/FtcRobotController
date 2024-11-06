package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import java.lang.Math;
import java.util.Base64;

@TeleOp
//@Disabled
public class GyroStraightFLL extends LinearOpMode {
    @Override
    public void runOpMode() {
        boolean robotMovedForward = false;
        boolean robotInMotion = false;
        double correction = 0;
        double leftSpeed = 0;
        double rightSpeed = 0;
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(myIMUparameters);
        imu.resetYaw();
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double startingPosition = backLeft.getCurrentPosition();
        waitForStart();

        while (opModeIsActive()) {
            double YawAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double driveTrainPoistion = backLeft.getCurrentPosition();
            YawAngle = (double) Math.round(YawAngle * 10) / 10.0;

            telemetry.addData("yawAngle",YawAngle);
            telemetry.addData("pos",driveTrainPoistion);
            telemetry.update();
            if (!robotMovedForward) {
                robotInMotion = true;
                correction = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                leftSpeed = 15 + correction;
                rightSpeed = 15 -correction;
                frontLeft.setPower(leftSpeed/100);
                frontRight.setPower(rightSpeed/100);
                backLeft.setPower(leftSpeed/100);
                backRight.setPower(rightSpeed/100);
            }
        }

    }

}
