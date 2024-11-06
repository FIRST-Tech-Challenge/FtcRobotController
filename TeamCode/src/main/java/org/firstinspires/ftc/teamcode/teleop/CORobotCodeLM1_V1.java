package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class CORobotCodeLM1_V1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");

        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");

        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");

        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        DcMotor rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");

        Servo rightWristServo = hardwareMap.get(Servo.class, "rightWristServo");

        Servo specServo = hardwareMap.get(Servo.class, "specServo");

        CRServo activeIntake = hardwareMap.get(CRServo.class, "activeIntake");

        DcMotor intakeArmMotor = hardwareMap.get(DcMotor.class, "intakeArmMotor");

        Servo bucketServo = hardwareMap.get(Servo.class, "bucketServo");

        //rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightSlideMotor.setTargetPosition(0);
        //rightSlideMotor.setPower(0.3);
        //rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeArmMotor.setTargetPosition(0);
        intakeArmMotor.setPower(0.3);
        intakeArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addLine("Start");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go for   wards,
        // reverse the left side instead.
        // See the note about this earlier on this page.

        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
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
//        rightSlideMotor.setPower(-0.5);
//        sleep(500);
//        intakeArmMotor.setTargetPosition(300);
//        sleep(1000);
//        rightSlideMotor.setPower(0);
//        sleep(500);

        rightSlideMotor.setPower(-0.5);
        sleep(3000);
        rightSlideMotor.setPower(-0.2);
        rightWristServo.setPosition(0.1);
        sleep(1000);
        intakeArmMotor.setTargetPosition(-420); //reset arm into position to make sure that existing teleop works
        sleep(1500);
        intakeArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoder for teleop
        intakeArmMotor.setTargetPosition(0);
        intakeArmMotor.setPower(0.2);
        intakeArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeArmMotor.setTargetPosition(300);
        sleep(1000);



        bucketServo.setPosition(0.27);


        if (isStopRequested()) return;

        double armServoReadyPos = 0.55;
        boolean lock = false;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;            // This button choice was made so that it is hard to hit on accident,
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


            if (gamepad1.y) {
                lock = true;
                intakeArmMotor.setTargetPosition(300);
                sleep(200);
                intakeArmMotor.setTargetPosition(380);
                sleep(500);
                rightWristServo.setPosition(armServoReadyPos);
                sleep(500);
                bucketServo.setPosition(0.27);
            }
            if (gamepad1.a) {
                intakeArmMotor.setTargetPosition(80);
                rightWristServo.setPosition(0.5);
                sleep(100);
                intakeArmMotor.setTargetPosition(80);
                rightWristServo.setPosition(0.8);
                sleep(100);
                rightWristServo.setPosition(0.43);
            }
            if (gamepad1.x) {
                rightWristServo.setPosition(0.5);
                intakeArmMotor.setTargetPosition(200);
            }
            if (gamepad2.a) {
                bucketServo.setPosition(0.27);
            }
            if (gamepad2.y) {
                bucketServo.setPosition(0.9);
            }
            if (gamepad1.right_bumper) {
                activeIntake.setPower(-1);
            }
            if (gamepad1.left_bumper) {
                activeIntake.setPower(0);
            }
            if (gamepad1.b) {
                activeIntake.setPower(1);
            }

            if (gamepad1.dpad_up){
                if (!lock){
                    armServoReadyPos = armServoReadyPos + 0.05;
                    lock = true;
                }
            }

            if (gamepad1.dpad_down){
                if (!lock){
                    armServoReadyPos = armServoReadyPos - 0.05;
                    lock = true;
                }
            }

            double power = gamepad2.left_stick_y;
            rightSlideMotor.setPower(power);

            //if (gamepad2.dpad_up){
            //  rightSlideMotor.setTargetPosition(624);
            //}
            if (gamepad2.b) {
                specServo.setPosition(0.2);
            }
            if (gamepad2.x) {
                specServo.setPosition(0.8);
            }
        }


    }

    private void waitForInput(){
//            printTelemetry("Waiting for gamepad1.a");
        while (true){
            if (gamepad1.a){
                return;
            }
            sleep(100);
        }
    }
}