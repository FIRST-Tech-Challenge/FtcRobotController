package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Debug.Debug;

@TeleOp(name = "TeleOpTest", group = "Test")
public class TeleOpTest extends LinearOpMode {
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;

    private DcMotor leftViper;
    private DcMotor rightViper;

    private double targetPower;
    private double increment;
    private double incrementDividend;
    private boolean isAccelerating;
    private long lastUpdateTime;
    private double currentPower;
    private int updateDelay = 10;

    float frontMultiplier = 1;
    float backMultiplier = 1;

    boolean emergencyStop = false;

    Debug debug;

    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        debug = new Debug(this);

//        leftViper = hardwareMap.get(DcMotor.class, "leftViper");
//        rightViper = hardwareMap.get(DcMotor.class, "rightViper");

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        leftViper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightViper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        leftViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

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

            startAccelerating(frontLeftPower / s, 1000, incrementDividend);
            startAccelerating(backLeftPower / s, 1000, incrementDividend);
            startAccelerating(frontRightPower / s, 1000, incrementDividend);
            startAccelerating(backRightPower / s, 1000, incrementDividend);

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

            // Slide
//            if (gamepad2.left_trigger != 0) {
//                leftViper.setPower(-gamepad2.left_trigger);
//                rightViper.setPower(gamepad2.left_trigger);
//            } else if (gamepad2.right_trigger != 0) {
//                leftViper.setPower(gamepad2.right_trigger);
//                rightViper.setPower(-gamepad2.right_trigger);
//            } else {
//                leftViper.setPower(0);
//                rightViper.setPower(0);
//            }

            // Debug
            debug.checkDebugButtons(gamepad2);


            accelerateMotors();
        }
    }

    private void stopAllMotors() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    private void startAccelerating(double newTargetPower, double duration, double incrementDividend) {
        if (isAccelerating) {
            // Adjust the increment based on the remaining time and power difference
            double remainingTime = duration - (System.currentTimeMillis() - lastUpdateTime);
            double powerDifference = newTargetPower - currentPower;
            this.increment = powerDifference / (remainingTime / incrementDividend);
        } else {
            this.targetPower = newTargetPower;
            this.increment = newTargetPower / (duration / incrementDividend);
            this.isAccelerating = true;
            this.lastUpdateTime = System.currentTimeMillis();
        }
    }

    private void accelerateMotors() {
        if (isAccelerating && !emergencyStop) {
            long currentTime = System.currentTimeMillis();
            if (currentTime - lastUpdateTime >= updateDelay) {
                currentPower += increment;
                if (currentPower >= targetPower) {
                    currentPower = targetPower;
                    isAccelerating = false;
                }
                frontLeft.setPower(currentPower);
                backLeft.setPower(currentPower);
                frontRight.setPower(currentPower);
                backRight.setPower(currentPower);
                lastUpdateTime = currentTime;
            }
        }
    }
}