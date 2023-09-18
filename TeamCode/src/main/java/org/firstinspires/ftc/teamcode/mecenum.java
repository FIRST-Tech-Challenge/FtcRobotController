package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Mecanum - DO")
public class mecenum extends LinearOpMode {

    public Servo Claw = null;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("motorBR");
        DcMotor liftMotor = hardwareMap.dcMotor.get("liftMotor");

        Claw = hardwareMap.get(Servo.class, "Servo1");

        // Reverse the right side motors. Flip if goes backward.
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        double clawPosition=0;

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.x) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            if (gamepad1.right_trigger>=0.4) {
                liftMotor.setPower(gamepad1.right_trigger);
                telemetry.addData("liftMotor.getCurrentPosition",liftMotor.getCurrentPosition());
            }
            else if (gamepad1.left_trigger>=0.4) {
                liftMotor.setPower(gamepad1.left_trigger * -1);
                telemetry.addData("liftMotor.getCurrentPosition",liftMotor.getCurrentPosition());
            }
            else {
                liftMotor.setPower(0);
            }

            if (gamepad2.right_trigger>=0.4) {
                liftMotor.setPower(gamepad1.right_trigger);
                telemetry.addData("liftMotor.getCurrentPosition",liftMotor.getCurrentPosition());
            }
            else if (gamepad2.left_trigger>=0.4) {
                liftMotor.setPower(gamepad1.left_trigger * -1);
                telemetry.addData("liftMotor.getCurrentPosition",liftMotor.getCurrentPosition());
            }
            else {
                liftMotor.setPower(0);
            }

            if (gamepad2.right_bumper && gamepad1.left_bumper) {
                telemetry.addLine("Claw Opened");
                clawPosition = 0.3;
                Claw.setPosition(clawPosition);
            }
            else if (gamepad2.right_bumper || gamepad1.left_bumper) {
                //don't need    telemetry.addData("gamepad1.right_bumper",gamepad1.right_bumper);
                telemetry.addLine("claw closed");
                clawPosition = 0.7;
                Claw.setPosition(clawPosition);
            }


        }
    }
}
