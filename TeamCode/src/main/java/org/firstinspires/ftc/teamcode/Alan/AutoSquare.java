package org.firstinspires.ftc.teamcode.Alan;

import android.util.Log;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous
public class AutoSquare extends LinearOpMode {
    private IMU imu;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor rightFront;

    private void goForward(double speed, int milliseconds) {
        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);
        rightFront.setPower(speed);
        sleep(milliseconds);
        leftFront.setPower(-speed);
        leftBack.setPower(-speed);
        rightBack.setPower(-speed);
        rightFront.setPower(-speed);
    }

    private void turnByYaw(double speed, double targetYaw) {
        imu.resetYaw();

        leftFront.setPower(-speed);
        leftBack.setPower(-speed);
        rightBack.setPower(speed);
        rightFront.setPower(speed);
        YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();
        double yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        while (yaw < targetYaw) {
            Log.d("alan", "yaw is currently: " + yaw);
            robotOrientation = imu.getRobotYawPitchRollAngles();
            yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        }
        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(-speed);
        rightFront.setPower(-speed);
    }

    @Override
    public void runOpMode() throws InterruptedException{
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                        )
                )
        );
        Log.d("alan", "reset yaw done");
        YawPitchRollAngles robotOrientation;
        leftFront = hardwareMap.dcMotor.get("fLeft");
        rightFront = hardwareMap.dcMotor.get("fRight");
        leftBack = hardwareMap.dcMotor.get("bLeft");
        rightBack = hardwareMap.dcMotor.get("bRight");
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        int count = 0;
        waitForStart();

        while (opModeIsActive()) {
            goForward(0.3, 1000);
            turnByYaw(0.1, 90);
            count++;
            if (count == 4) {
                break;
            }
        }
    }
}
