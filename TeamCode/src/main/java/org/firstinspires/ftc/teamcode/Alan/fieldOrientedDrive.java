package org.firstinspires.ftc.teamcode.Alan;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
/*
Field oriented Drive!!!!!!!!
 */
@TeleOp
public class fieldOrientedDrive extends LinearOpMode {
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    /*
    Thats too much bacon

    Please just bring me some kale chips

    said no one ever
     */
    private IMU imu;
    Orientation angles = new Orientation();

    double initYaw;
    double adjustedYaw;
    public void runOpMode() {

        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                )
        );
        angles = imu.getRobotOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);

        initYaw = angles.firstAngle;
        /*
        Haiku's are easy

        but some times they don't make sense

        refrigerator
         */

        leftFront = hardwareMap.get(DcMotor.class,"fLeft");
        leftBack = hardwareMap.get(DcMotor.class,"bLeft");
        rightFront = hardwareMap.get(DcMotor.class,"fRight");
        rightBack = hardwareMap.get(DcMotor.class,"bRight");

        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);;;;;;;;;;
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);;;;;;;;;;;;;;
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);;;;;;;;;;
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);;;;;;;;;;;;;;;;;;;
        imu.resetYaw();;;;;;;;;;;;;;;;;;;;;
        waitForStart();;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        while (opModeIsActive()) {

            YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();;;;;;;;;;;;;;;;;;;;;
            double yaw = robotOrientation.getYaw(AngleUnit.DEGREES);;;;;;;;;;;;;;

            angles=imu.getRobotOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX, AngleUnit.DEGREES);;;;;;;;;;;;;;;;;;;;;;

            adjustedYaw = angles.firstAngle-initYaw;;;;;;;;;;;;;;;;
            double zerodYaw = yaw;;;;;;;;;;;;;;;;;;;;

            double y = gamepad1.left_stick_x;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
            double x = -gamepad1.left_stick_y;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
            double turn = gamepad1.right_stick_x;;;;;;;;;;;;;;;;;;;;;;;;;;

            double theta = Math.atan2(x, y) * 180/Math.PI;;;;;;;;;;;;;;;;;;
            double realTheta;;;;;;;;;;;;;;;;;;;;;;;;;;
            realTheta = (360 - zerodYaw) + theta;;;;;;;;;;;;;;;;;;;;;;;;;;;;
            double power = Math.hypot(x, y);;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

            double sin = Math.sin((realTheta * (Math.PI / 180)) - (Math.PI / 4));;;;;;;;;;;;;;;;;;;;;;
            double cos = Math.cos((realTheta * (Math.PI / 180)) - (Math.PI / 4));;;;;;;;;;;;;;;;
            double maxSinCos = Math.max(Math.abs(sin), Math.abs(cos));;;;;;;;;;;;;;;;;;;

            double leftFrontPower = (power * cos / maxSinCos + turn);;;;;;;;;;;;;;;;
            double rightFrontPower = (power * sin / maxSinCos - turn);;;;;;;;;;;;;;;;;;;;;;;;;;
            double leftBackPower = (power * sin / maxSinCos + turn);;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
            double rightBackPower = (power * cos / maxSinCos - turn);
            if ((power + Math.abs(turn)) > 1) {
                leftFrontPower /= power + turn;
                rightFrontPower /= power - turn;
                leftBackPower /= power + turn;
                rightBackPower /= power - turn;
            }
            /*
            cereal

            cereal

            cereal
             */
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
            robotOrientation = imu.getRobotYawPitchRollAngles();
            yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
            Log.d("alan", "" + yaw);
        }
    }
}
