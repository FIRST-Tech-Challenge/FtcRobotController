package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Debug.Debug;

@TeleOp(name = "TeleOpMain", group = "Main")
public class TeleOpMain extends LinearOpMode {
    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backRight;

    private DcMotorEx leftViper;
    private DcMotorEx rightViper;

    private DcMotorEx slideMotor;

    private Servo leftBucket;
    private Servo rightBucket;

    private Servo leftWrist;
    private Servo rightWrist;

    private Servo leftGrabber;
    private Servo rightGrabber;

    private double targetPower;
    private double increment;
    private double incrementDividend;
    private boolean isAccelerating;
    private long lastUpdateTime;
    private double currentPower;
    private int updateDelay = 10;

    float frontMultiplier = 1;
    float backMultiplier = 1;

    boolean debugMode = false;
    boolean emergencyStop = false;

    Debug debug;

    @Override
    public void runOpMode() {
        // Initialize motors and servos
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        leftViper = hardwareMap.get(DcMotorEx.class, "leftViper");
        rightViper = hardwareMap.get(DcMotorEx.class, "rightViper");

        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");

        leftBucket = hardwareMap.get(Servo.class, "leftBucket");
        rightBucket = hardwareMap.get(Servo.class, "rightBucket");

        leftWrist = hardwareMap.get(Servo.class, "leftWrist");
        rightWrist = hardwareMap.get(Servo.class, "rightWrist");

        leftGrabber = hardwareMap.get(Servo.class, "leftGrabber");
        rightGrabber = hardwareMap.get(Servo.class, "rightGrabber");

        // Motor and servo setup
        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftViper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightViper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftViper.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightViper.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);

        leftWrist.setPosition(0);
        rightWrist.setPosition(1);

        leftBucket.setPosition(1);
        rightBucket.setPosition(0);

        double direction = 1;

        int horizontalSlideLimit = 550;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        debug = new Debug(this);



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

            // slow speeds
            if (gamepad1.left_trigger != 0)
                s = 4;
            else if (gamepad1.right_trigger != 0)
                s = 2;
            else
                s = 1;

            frontLeft.setPower(frontLeftPower / s);
            backLeft.setPower(backLeftPower / s);
            frontRight.setPower(frontRightPower / s);
            backRight.setPower(backRightPower / s);


            // Switching directions
            if (gamepad1.y) {
                direction = 1;
                frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
                backLeft.setDirection(DcMotorEx.Direction.REVERSE);
                frontRight.setDirection(DcMotorEx.Direction.FORWARD);
                backRight.setDirection(DcMotorEx.Direction.FORWARD);
            } else if (gamepad1.a) {
                direction = -1;
                frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
                backLeft.setDirection(DcMotorEx.Direction.FORWARD);
                frontRight.setDirection(DcMotorEx.Direction.REVERSE);
                backRight.setDirection(DcMotorEx.Direction.REVERSE);
            }

            // Viper Slide
            if (gamepad2.left_trigger != 0) {
                leftViper.setPower(gamepad2.left_trigger);
                rightViper.setPower(-gamepad2.left_trigger);
            } else if (gamepad2.right_trigger != 0) {
                leftViper.setPower(-gamepad2.right_trigger);
                rightViper.setPower(gamepad2.right_trigger);
            } else {
                leftViper.setPower(0);
                rightViper.setPower(0);
            }

            // Horizontal slide
            //todo auto must reset to zero
            if (gamepad1.right_bumper && slideMotor.getCurrentPosition() < 550) {
                slideMotor.setPower(1);
            } else if (gamepad1.left_bumper && slideMotor.getCurrent(CurrentUnit.AMPS) < 2) {

                slideMotor.setPower(-1);
            } else {
                if(slideMotor.getCurrentPosition() >= 547) {
                    telemetry.addData("Limit reached, ", "Stopped");
                }
                slideMotor.setPower(0);
            }
            telemetry.update();

            // Wrist
            // SERVO CONTROLLER INFO: leftWrist:        left limit = 1, right limit = 0     rightWrist: left limit = 0, right limit = 1
            if (gamepad1.b) {           // pick up
                leftWrist.setPosition(0.9);
                rightWrist.setPosition(0.1);
            } else if (gamepad1.x) {      // deposit
                leftWrist.setPosition(0);
                rightWrist.setPosition(1);
            }

            // Grabber
            if (gamepad1.dpad_up) {
                leftGrabber.setPosition(0);
                rightGrabber.setPosition(1);
            }
            else if (gamepad1.dpad_down) {
                leftGrabber.setPosition(1);
                rightGrabber.setPosition(0);
            }
            else {
                leftGrabber.setPosition(0.5);
                rightGrabber.setPosition(0.5);
            }

            // Bucket
            if (gamepad2.y) {           // receiving
                leftBucket.setPosition(1);
                rightBucket.setPosition(0);
            } else if (gamepad2.x) {      // scoring
                leftBucket.setPosition(0.55);
                rightBucket.setPosition(0.45);
            }



            // Debug

            debug.checkDebugButtons(gamepad1);


//            if (gamepad1.start && gamepad1.back) {
//                debugMode = !debugMode;
//                telemetry.addData("debug mode: ", debugMode);
//                telemetry.update();
//            }

            // Fixed this
            // Used to only check this while the debug mode buttons were being pressed
//            if (debugMode) {
//
//                if(gamepad2.y) {
//                    if(gamepad2.dpad_up) {
//
//                    }
//                }

//                if (gamepad2.y) { // Front wheels
//                    telemetry.addData("editing: ", "front wheels");
//                    if (gamepad2.dpad_up) {
//                        frontMultiplier += 0.1;
//                        telemetry.addData("frontMultiplier: ", frontMultiplier);
//                        telemetry.update();
//                    } else if (gamepad2.dpad_down) {
//                        frontMultiplier -= 0.1;
//                        telemetry.addData("frontMultiplier: ", frontMultiplier);
//                        telemetry.update();
//                    }
//                } else if (gamepad2.a) { // Back wheels
//                    telemetry.addData("editing: ", "back wheels");
//                    if (gamepad2.dpad_up) {
//                        backMultiplier += 0.1;
//                        telemetry.addData("backMultiplier: ", backMultiplier);
//                        telemetry.update();
//                    } else if (gamepad2.dpad_down) {
//                        backMultiplier -= 0.1;
//                        telemetry.addData("backMultiplier: ", backMultiplier);
//                        telemetry.update();
//                    }
//                }
            }

            // Update motor powers
//        }
    }

    private void stopAllMotors() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        leftViper.setPower(0);
        rightViper.setPower(0);
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