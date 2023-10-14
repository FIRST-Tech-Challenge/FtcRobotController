package org.firstinspires.ftc.teamcode.versionCode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "V3 Claw Test Code")
public class V3_2023_10_10_01_MecanumLiftWristClawDrone extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("motorFL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("motorBL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("motorFR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("motorBR");

        DcMotor lift = hardwareMap.dcMotor.get("lift");
        Servo droneServo = hardwareMap.servo.get("drone");
        Servo wristServo = hardwareMap.servo.get("wrist");
        Servo clawServo = hardwareMap.servo.get("claw");

        // Reverse the right side motors. Flip if goes backward.
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        droneServo.setDirection(Servo.Direction.FORWARD);
        wristServo.setDirection(Servo.Direction.FORWARD);
        clawServo.setDirection(Servo.Direction.FORWARD);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB backward
        imu.initialize(parameters);

        droneServo.scaleRange(0, 1);
        droneServo.setPosition(0.85);
        wristServo.setPosition(0);
        clawServo.scaleRange(0.3, 0.7);
        clawServo.setPosition(0);

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            int targetPosition = 0;
            double wristTargetPos = 0;

            double droneServoPosition = droneServo.getPosition();
            double liftPower = lift.getPower();
            double liftPos = lift.getCurrentPosition();

            if (gamepad1.right_trigger != 0) {
                targetPosition += 10;
            }else if (gamepad1.left_trigger != 0) {
                targetPosition -= 10;
            }
            if (targetPosition > 1000){
                targetPosition = 990;
            }
            else if (targetPosition < 0){
                targetPosition = 10;
            }
            lift.setTargetPosition(targetPosition);
            lift.setPower(0.3);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (wristTargetPos > 1){
                wristTargetPos = 0.9;
            } else if (wristTargetPos < 0){
                wristTargetPos = 0.1;
            }
            if (gamepad1.right_bumper) {
                wristTargetPos += 0.1;
                wristServo.setPosition(wristTargetPos);
            }else if (gamepad1.left_bumper) {
                wristTargetPos -= 0.1;
                wristServo.setPosition(wristTargetPos);
            }

            if (gamepad1.y) {

            }

            if (gamepad1.a) {
                droneServo.setPosition(1);
                sleep(1500);
                droneServo.setPosition(0.85);
            }

            // ADDED CODE - sends info about current servo position to driver station
            telemetry.addData("Servo Position: ", droneServoPosition);
            telemetry.addData("Wrist Position: ", wristServo.getPosition());
            telemetry.addData("Lift position: ", liftPos);
            telemetry.addData("Lift power: ", liftPower);
            telemetry.update();

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
            double frontLeftPower = ((rotY + rotX + rx) / denominator);
            double backLeftPower = ((rotY - rotX + rx) / denominator);
            double frontRightPower = ((rotY - rotX - rx) / denominator);
            double backRightPower = ((rotY + rotX - rx) / denominator);

            frontLeftMotor.setPower(frontLeftPower/2);
            backLeftMotor.setPower(backLeftPower/2);
            frontRightMotor.setPower(frontRightPower/2);
            backRightMotor.setPower(backRightPower/2);

        }
    }
}
