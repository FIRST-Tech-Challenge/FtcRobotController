package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TestBot", group = "Test")
public class TestBotMain extends LinearOpMode {
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;

    boolean debugMode = false;
    boolean emergencyStop = false;
    double speedMultiplier = 1.0;
    double targetSpeedMultiplier = 1.0;
    double accelerationRate = 0.01; // Adjust this value to control acceleration speed

    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        double direction = 1;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !emergencyStop) {
            // Emergency stop
            if (gamepad1.dpad_left && gamepad1.b) {
                emergencyStop = true;
                stopAllMotors();
                telemetry.addData("Emergency Stop", "Activated");
                telemetry.update();
                break;
            }

            // Gamepad 1 controls
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            if (direction == 1) {
                rx = gamepad1.right_stick_x;
            } else {
                rx = gamepad1.right_stick_x * -1;
            }

            // TODO toggleable deadzones
            if (gamepad1.left_stick_x < 40 && gamepad1.left_stick_x > 60) {
                x = 50;
            }
            if (gamepad1.left_stick_y < 45 && gamepad1.left_stick_y > 55) {
                y = 50;
            }

            int s = 1;
            if (gamepad1.left_bumper)
                s = 4;
            else if (gamepad1.right_bumper)
                s = 2;
            else
                s = 1;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // Update target speed multiplier based on s
            targetSpeedMultiplier = 1.0 / s;

            // Update speed multiplier for acceleration
            if (speedMultiplier < targetSpeedMultiplier) {
                speedMultiplier += accelerationRate;
                if (speedMultiplier > targetSpeedMultiplier) {
                    speedMultiplier = targetSpeedMultiplier;
                }
            } else if (speedMultiplier > targetSpeedMultiplier) {
                speedMultiplier -= accelerationRate;
                if (speedMultiplier < targetSpeedMultiplier) {
                    speedMultiplier = targetSpeedMultiplier;
                }
            }

            // Update motor powers with speed multiplier
            frontLeft.setPower(frontLeftPower * speedMultiplier);
            backLeft.setPower(backLeftPower * speedMultiplier);
            frontRight.setPower(frontRightPower * speedMultiplier);
            backRight.setPower(backRightPower * speedMultiplier);

            // Switching directions
            if (gamepad1.y) {
                direction = 1;
                frontLeft.setDirection(DcMotor.Direction.REVERSE);
                backLeft.setDirection(DcMotor.Direction.REVERSE);
                frontRight.setDirection(DcMotor.Direction.FORWARD);
                backRight.setDirection(DcMotor.Direction.FORWARD);
            } else if (gamepad1.b) {
                direction = -1;
                frontLeft.setDirection(DcMotor.Direction.FORWARD);
                backLeft.setDirection(DcMotor.Direction.FORWARD);
                frontRight.setDirection(DcMotor.Direction.REVERSE);
                backRight.setDirection(DcMotor.Direction.REVERSE);
            }

            // Debug
            if (gamepad2.start && gamepad2.back) {
                telemetry.addData("debug mode: ", debugMode);
                telemetry.update();
                debugMode = !debugMode;
            }

            if(debugMode) {
//                telemetry.addData("frontLeftPower", frontLeftPower);
//                telemetry.addData("backLeftPower", backLeftPower);
//                telemetry.addData("frontRightPower", frontRightPower);
//                telemetry.addData("backRightPower", backRightPower);
//                telemetry.addData("speedMultiplier", speedMultiplier);
//                telemetry.addData("targetSpeedMultiplier", targetSpeedMultiplier);

                if(gamepad2.y) { // fine tune accelerationRate
                    if(gamepad2.dpad_up) {
                        accelerationRate += 0.01;
                        telemetry.addData("accelerationRate", accelerationRate);
                        System.out.println("accelerationRate: " + accelerationRate);
                        telemetry.update();
                    } else if(gamepad2.dpad_down) {
                        accelerationRate -= 0.01;
                        telemetry.addData("accelerationRate", accelerationRate);
                        System.out.println("accelerationRate: " + accelerationRate);
                        telemetry.update();
                    }
                }

                if(gamepad2.a) { // fine tune targetSpeedMultiplier
                    if(gamepad2.dpad_up) {
                        targetSpeedMultiplier += 0.01;
                        telemetry.addData("targetSpeedMultiplier", targetSpeedMultiplier);
                        telemetry.update();
                    } else if(gamepad2.dpad_down) {
                        targetSpeedMultiplier -= 0.01;
                        telemetry.addData("targetSpeedMultiplier", targetSpeedMultiplier);
                        telemetry.update();
                    }
                }
            }
        }
    }

    private void stopAllMotors() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }
}