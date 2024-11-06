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
public class CORobotCodeV6 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");
        DcMotor leftSlideMotor = hardwareMap.get(DcMotor.class, "leftSlideMotor");
        Servo rightWristServo = hardwareMap.get(Servo.class, "rightWristServo");
        Servo leftWristServo = hardwareMap.get(Servo.class, "leftWristServo");
        Servo rightClawServo = hardwareMap.get(Servo.class, "rightClawServo");
        Servo leftClawServo = hardwareMap.get(Servo.class, "leftClawServo");
        DcMotor intakeArmMotor = hardwareMap.get(DcMotor.class, "intakeArmMotor");
        Servo bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        intakeArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

        intakeArmMotor.setTargetPosition(0);
        leftSlideMotor.setTargetPosition(0);
        rightSlideMotor.setTargetPosition(0);

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
//Linear Slide movement
                if (gamepad2.b) {
                    rightSlideMotor.setTargetPosition(3764);
                    leftSlideMotor.setTargetPosition(3764);
                    if (gamepad2.a) {
                        rightSlideMotor.setTargetPosition(0);
                        leftSlideMotor.setTargetPosition(0);
                        // Linear Slide movement End

                            if (gamepad1.left_trigger == 1) {
                                telemetry.addLine("lefttrigger pressed");
                                rightClawServo.setPosition(1);
                                leftClawServo.setPosition(1);
                            if (gamepad1.right_trigger == 1) {
                                telemetry.addLine("righttrigger pressed");
                                rightClawServo.setPosition(0);
                                leftClawServo.setPosition(0);

                                        if (gamepad2.x) {
                                            telemetry.addLine("x pressed");
                                            rightWristServo.setPosition(-0.25);
                                            leftWristServo.setPosition(-0.25);
                                            sleep(200);
                                            intakeArmMotor.setTargetPosition(-134);


                                            if (gamepad2.a) {
                                                telemetry.addLine("a pressed");
                                                intakeArmMotor.setTargetPosition(0);
                                                sleep(200);
                                                rightWristServo.setPosition(0.25);
                                                leftWristServo.setPosition(0.25);

                                                if (gamepad1.y) {
                                                    telemetry.addLine("y pressed");
                                                    bucketServo.setPosition(-0.5);
                                                }

                                                if (gamepad1.b) {
                                                    telemetry.addLine("b pressed");
                                                    bucketServo.setPosition(0);

                                                }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }









                        }
                    }
