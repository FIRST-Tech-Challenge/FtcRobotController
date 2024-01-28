package org.firstinspires.ftc.teamcode;

import  com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp (name = "TeleOpCode")
public class TeleOpCode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        DcMotor leftHangingMotor = hardwareMap.dcMotor.get("leftHangingMotor");
        DcMotor rightHangingMotor = hardwareMap.dcMotor.get("rightHangingMotor");

        DcMotor armMotor = hardwareMap.dcMotor.get("armMotor");

        CRServo claw;
        CRServo arm;
        Servo drone;

        claw = hardwareMap.get(CRServo.class, "claw");
        arm = hardwareMap.get(CRServo.class, "arm");
        drone = hardwareMap.get(Servo.class, "drone");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
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



            if(gamepad1.a){
                rightHangingMotor.setPower(-0.4);
                leftHangingMotor.setPower(0.4);
            }

            if(gamepad1.b) {
                rightHangingMotor.setPower(0.7);
                leftHangingMotor.setPower(-0.7);
            }
// Likely the reason this wasn't working is because its probably not called dpad
//            while(gamepad2.x){
//                armMotor.setPower(1);
//            }
//            while(gamepad2.y){
//                armMotor.setPower(-1);
//            }
//            armMotor.setPower(0);


            //The reason the claw wasn't activating is because the cable was in backwards
            while(gamepad2.dpad_right){
                claw.setPower(1);
            }
            while(gamepad2.dpad_left){
                claw.setPower(-1);
            }
            claw.setPower(0);


            //The reason the motor wasn't workign is still
//            if (gamepad2.a) {
//                arm.setPower(0.5);
//            } else if (gamepad2.b) {
//                arm.setPower(-0.5);
//            } else {
//                arm.setPower(-0.1);
//            }
//            while(gamepad2.a){
//                arm.setPower(0.5);
//            }
//            while(gamepad2.b){
//                arm.setPower(-0.5);
//            }
//            arm.setPower(0);

            while(gamepad1.dpad_down){
                drone.setPosition(1);
            }
            while(gamepad1.dpad_up){
                drone.setPosition(-1);
            }
            drone.setPosition(0);
//            while(gamepad2.x){
//                drone.setPosition(1);
//            }


        }
    }
}