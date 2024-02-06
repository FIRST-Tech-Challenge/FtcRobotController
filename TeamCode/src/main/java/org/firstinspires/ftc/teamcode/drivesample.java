package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
public class drivesample extends LinearOpMode {

    private void wristServo(double pos, Servo s1, Servo s2) {
        s1.setPosition(pos);
        s2.setPosition(pos);
    }

    private void setArmPos(double degrees, Servo arm) {
        degrees = (degrees + 360) % 360;
        // TODO: some algorithm to get the 0-1 value for the servo
        double pos = 0;
        arm.setPosition(pos);
    }

    private void setArmWristPos(double degrees, Servo armWrist1, Servo armWrist2) {
        degrees = (degrees + 360) % 360;
        // TODO: some algorithm to get the 0-1 value for the servo
        double pos = 0;
        wristServo(pos, armWrist1, armWrist2);
    }

    private void armInverseKinematics(double x, double y, Servo arm, Servo armWrist1, Servo armWrist2) {
        double armLength1 = 3;
        double armLength2 = 2;
        double distance = Math.hypot(x, y);

        double phi = Math.atan(y / x);
        double theta = Math.acos((Math.pow(armLength1, 2) + Math.pow(distance, 2) - Math.pow(armLength2, 2)) / 2 * armLength1 * distance);
        double alpha = Math.acos((Math.pow(armLength1, 2) + Math.pow(armLength2, 2) - Math.pow(distance, 2)) / 2 * armLength1 * armLength2);

        setArmPos(phi + theta, arm);
        setArmWristPos(alpha - 180, armWrist1, armWrist2);
    }

    RevBlinkinLedDriver lights;
    public enum State {
        HIGH_2ND_LINE,
        LOW,
        MID,
        RESET,
        GRAB
    }

    @Override
    public void runOpMode() throws InterruptedException {
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        DcMotor slideR = hardwareMap.dcMotor.get("slideRight");
        DcMotor slideL = hardwareMap.dcMotor.get("slideLeft");


        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideR.setDirection(DcMotorSimple.Direction.FORWARD);
        slideL.setDirection(DcMotorSimple.Direction.REVERSE);

        State state = State.RESET;

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Goof"))
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)
                .enableLiveView(true)
                .build();

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        imu.resetYaw();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //SECOND PLAYER
//            switch (state) {
//                case RESET:
//                    // servo.setpos(0)
//                    if (gamepad2.b) {
//                        state = State.LOW;
//                    } else if (gamepad2.y) {
//                        state = State.HIGH_2ND_LINE;
//                    } else if (gamepad2.x) {
//                        state = State.MID;
//                    } else if (gamepad2.right_bumper) {
//                        state = State.GRAB;
//                    }
//                    break;
//                case GRAB:
//                    if
//            }

            //FIRST PLAYER
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;


            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            if (gamepad1.y) {
                slideL.setPower(1);
                slideR.setPower(1);
            }
            if (gamepad1.a) {
                slideL.setPower(-1);
                slideR.setPower(-1);
            }
            if (gamepad1.b) {
                slideL.setPower(0);
                slideR.setPower(0);
            if (gamepad1.x){
                slideL.setPower(0.2);
                slideR.setPower(0.2);
            }

            }
        }
    }
}