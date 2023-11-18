package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class drivesample extends LinearOpMode {
    RevBlinkinLedDriver lights;
    @Override
    public void runOpMode() throws InterruptedException {
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("FLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("RLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("FRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("RRight");
        DcMotor intakemotor = hardwareMap.dcMotor.get("spinwoo");
        Servo arm = hardwareMap.get(Servo.class, "intaker");
        Servo wrist = hardwareMap.get(Servo.class, "hahah");
        Servo out = hardwareMap.get(Servo.class, "armo");
        Servo bla = hardwareMap.get(Servo.class, "grab");
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bla.setDirection(Servo.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        out.setPosition(0.1);
        arm.setPosition(0);
        bla.setPosition(0.5);
        wrist.setPosition(0.2);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
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
            if(gamepad1.dpad_down){
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE);
            }
            if(gamepad1.dpad_up){
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }
            if(gamepad1.dpad_left){
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            }
            if(gamepad1.dpad_right){
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            }
            if(gamepad1.right_bumper){
                arm.setPosition(0.55);
                intakemotor.setPower(1);
                out.setPosition(1);
                wrist.setPosition(0);

            }
            if(gamepad1.left_bumper){
                arm.setPosition(0);
                intakemotor.setPower(0);
            }
            if(gamepad1.b){
                out.setPosition(1);
                intakemotor.setPower(0);
                arm.setPosition(0.3);
                wrist.setPosition(0);



            }
            if(gamepad1.x){
                wrist.setPosition(0.23);

                out.setPosition(0.1);
            }
            if(gamepad1.y){
                bla.setPosition(0.5);
            }
            if(gamepad1.a){
                bla.setPosition(0.2);
            }

        }
    }
}