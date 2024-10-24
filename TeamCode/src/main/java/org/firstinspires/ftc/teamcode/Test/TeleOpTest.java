
package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOpTest", group = "Test")

public class TeleOpTest extends LinearOpMode {
    // TODO change to dcmotorex
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    private DcMotor leftViper;
    private DcMotor rightViper;

    float frontMultiplier = 1;
    float backMultiplier = 1;

    boolean debugMode = false;

    @Override
    public void runOpMode() {

        // initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        leftViper = hardwareMap.get(DcMotor.class, "leftViper");
        rightViper = hardwareMap.get(DcMotor.class, "rightViper");





        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftViper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightViper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // reverse motors
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        double direction = 1;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // gamepad 1 controls
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            int accelerationMultiplier = 1;

            if(direction == 1) {
                rx = gamepad1.right_stick_x;
            }
            else {
                rx = gamepad1.right_stick_x * -1;
            }

            // TODO togglable deadzones
            if(gamepad1.left_stick_x < 40 && gamepad1.left_stick_x > 60) {
                x = 50;
            }
            if(gamepad1.left_stick_y < 45 && gamepad1.left_stick_y > 55) {
                y = 50;
            }

            int s = 1;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            if (gamepad1.left_bumper)
                s = 4;
            else if (gamepad1.right_bumper)
                s = 2;
            else
                s = 1;

            frontLeft.setPower((frontLeftPower / s) * frontMultiplier);
            backLeft.setPower(backLeftPower / s * backMultiplier);
            frontRight.setPower(frontRightPower / s * frontMultiplier);
            backRight.setPower(backRightPower / s * backMultiplier);

            // switching directions
            if(gamepad1.y) {
                direction = 1;
                frontLeft.setDirection(DcMotor.Direction.REVERSE);
                backLeft.setDirection(DcMotor.Direction.REVERSE);
                frontRight.setDirection(DcMotor.Direction.FORWARD);
                backRight.setDirection(DcMotor.Direction.FORWARD);
            }
            else if(gamepad1.b) {
                direction = -1;
                frontLeft.setDirection(DcMotor.Direction.FORWARD);
                backLeft.setDirection(DcMotor.Direction.FORWARD);
                frontRight.setDirection(DcMotor.Direction.REVERSE);
                backRight.setDirection(DcMotor.Direction.REVERSE);
            }


            // slide

            if(gamepad2.left_trigger != 0) {
                leftViper.setPower(-gamepad2.left_trigger);
                rightViper.setPower(gamepad2.left_trigger);
            }
            else if(gamepad2.right_trigger != 0) {
                leftViper.setPower(gamepad2.right_trigger);
                rightViper.setPower(-gamepad2.right_trigger);
            }
            else {
                leftViper.setPower(0);
                rightViper.setPower(0);
            }

            // debug
            if(gamepad2.start && gamepad2.back) {
                telemetry.addData("debug mode: ", debugMode);
                telemetry.update();
                debugMode = !debugMode;

                if(gamepad2.y) { // front wheels
                    telemetry.addData("editing: ", "front wheels");
                    if (gamepad2.dpad_up) {
                        frontMultiplier += 0.1;
                        telemetry.addData("frontMultiplier: ", frontMultiplier);
                        telemetry.update();
                    } else if (gamepad2.dpad_down) {
                        frontMultiplier -= 0.1;
                        telemetry.addData("frontMultiplier: ", frontMultiplier);
                        telemetry.update();
                    }
                }
                else if(gamepad2.a) { // back wheels
                    telemetry.addData("editing: ", "back wheels");
                    if(gamepad2.dpad_up) {
                        backMultiplier += 0.1;
                        telemetry.addData("backMultiplier: ", backMultiplier);
                        telemetry.update();
                    }
                    else if(gamepad2.dpad_down) {
                        backMultiplier -= 0.1;
                        telemetry.addData("backMultiplier: ", backMultiplier);
                        telemetry.update();
                    }
                }
            }

        }
    }
}