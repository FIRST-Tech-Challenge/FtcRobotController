package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;



@TeleOp(name = "IntoTheDeepTeleop", group = "Drive")
public class IntoTheDeepTeleop extends OpMode {
    // Declare thingies
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx leftLiftMotor, rightLiftMotor;
    private DcMotorEx intakeSlidesMotor;
    private Servo basketServo, intakePivotServo;
    private CRServo intakeServoLeft, intakeServoRight;
    private TouchSensor intakeTouchSensor;
    boolean liftHoldingTrigger = false; // flag to determine if lift trigger is being held
    boolean posUpdated = true; // flag to determine if lift position has been updated
    int liftPosTier = 0; // 0 = bottom, 1 = middle, 2 = top
    boolean intakeFull = false; // flag to determine if intake is full
    boolean isRetracting = false; // flag to determine if horizontal slides are retracting

    @Override
    public void init() {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        intakeSlidesMotor = hardwareMap.get(DcMotorEx.class, "intakeSlidesMotor");

        // Set motors' directions if needed
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        // Set motors to use encoders
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Servo for basket
        basketServo = hardwareMap.get(Servo.class, "basketServo");

        // Initialize lift motors
        leftLiftMotor = hardwareMap.get(DcMotorEx.class, "leftLiftMotor");
        rightLiftMotor = hardwareMap.get(DcMotorEx.class, "rightLiftMotor");

        // intake servos
        intakePivotServo = hardwareMap.get(Servo.class, "intakePivotServo");
        intakeServoLeft = hardwareMap.get(CRServo.class, "intakeServoLeft");
        intakeServoRight = hardwareMap.get(CRServo.class, "intakeServoRight");
        intakeTouchSensor = hardwareMap.get(TouchSensor.class, "intakeTouchSensor");

        // Set lift motors to use encoders
        leftLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        intakeSlidesMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlidesMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set all motors to zero power
        setMotorPower(0, 0, 0, 0);

        // initialize basket servo to 0
        basketServo.setPosition(0.0);

        // initialize intake pivot servo to 0
        intakePivotServo.setPosition(0.0);
    }

    @Override
    public void loop() {
        // Get joystick values
        double boost = gamepad1.right_trigger / 2; // Boost
        double y = 0.0;
        double x = 0.0;

        if (gamepad1.dpad_up){
            y = 0.5;
        }else if (gamepad1.dpad_down){
            y = -0.5;
        }else if (gamepad1.dpad_left){
            x = -0.5;
        }else if (gamepad1.dpad_right){
            x = 0.5;
        }else {
            y = -gamepad1.left_stick_y / 2; // Forward/backward
            x = gamepad1.left_stick_x / 2; // Strafe
        }

        double rotation = gamepad1.right_stick_x; // Rotate
        y += y * boost;
        x += x * boost;


        // Mecanum drive calculations
        double frontLeftPower = (y + x + rotation);
        double frontRightPower = (y - x - rotation);
        double backLeftPower = (y - x + rotation);
        double backRightPower = (y + x - rotation);

        // Normalize the power values to be within -1 and 1
        frontLeftPower /= Math.max(1.0, Math.abs(frontLeftPower));
        frontRightPower /= Math.max(1.0, Math.abs(frontRightPower));
        backLeftPower /= Math.max(1.0, Math.abs(backLeftPower));
        backRightPower /= Math.max(1.0, Math.abs(backRightPower));




        // Set motor power
        setMotorPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);

        // Check for lift and basket inputs
        lift();
        basket();
        intake();
        horizontalSlides();

        // Display encoder tick values for each lift motor
        telemetry.addData("Left Lift Motor Position:", leftLiftMotor.getCurrentPosition());
        telemetry.addData("Right Lift Motor Position:", rightLiftMotor.getCurrentPosition());
        telemetry.addData("Horizontal Slides Position:", intakeSlidesMotor.getCurrentPosition());
        telemetry.update();
    }

    public void basket() {
        if (gamepad2.b && (liftPosTier != 0)) {
            basketServo.setPosition(1.0);
        } else {
            basketServo.setPosition(0.0);
        }
    }

    public void setLiftPosition(int targetPosition) {
        // Strafing configuration for mecanum wheels
        leftLiftMotor.setTargetPosition(-targetPosition);
        rightLiftMotor.setTargetPosition(targetPosition);
        leftLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftLiftMotor.setVelocity(2000);
        rightLiftMotor.setVelocity(2000);
    }

    private void lift() {

        if (gamepad2.right_bumper && !liftHoldingTrigger && !(liftPosTier >= 2)) {
            liftHoldingTrigger = true;
            posUpdated = false;
            liftPosTier += 1;
        } else if (gamepad2.left_bumper && !liftHoldingTrigger && !(liftPosTier <= 0)) {
            liftHoldingTrigger = true;
            posUpdated = false;
            liftPosTier -= 1;
        } else if (!gamepad2.right_bumper && !gamepad2.left_bumper) {
            liftHoldingTrigger = false;
        }

        // handle setting lift position based on requested tier
        if (!posUpdated) {
            if (liftPosTier == 0) {
                setLiftPosition(0);
            } else if (liftPosTier == 1) {
                setLiftPosition(1659); // high basket encoder positions
            } else if (liftPosTier == 2) {
                setLiftPosition(3266); // low basket encoder positions
            }
            posUpdated = true;
        }
    }

    private void setMotorPower(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    // sets horizontal slides target position to 0
    public void retractHorizontalSlides() {
        intakeSlidesMotor.setTargetPosition(0);
        intakeSlidesMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeSlidesMotor.setVelocity(1000);

        isRetracting = true;
    }

    public enum IntakeMode {
        NORMAL,
        REVERSE,
        STOP
    }

    public void runIntakeMotors(IntakeMode mode) {
        switch (mode) {
            case NORMAL:
                intakeServoLeft.setPower(-1.0);
                intakeServoRight.setPower(1.0);
                break;
            case REVERSE:
                intakeServoLeft.setPower(1.0);
                intakeServoRight.setPower(-1.0);
                break;
            case STOP:
                intakeServoLeft.setPower(0.0);
                intakeServoRight.setPower(0.0);
                break;
        }
    }

    public void intake() {

        // intake is full or override button (a) has been pressed, stop running servos and retract to deposit in basket
        if ((intakeTouchSensor.isPressed() || gamepad2.a) && !intakeFull) {
            intakeFull = true;

            runIntakeMotors(IntakeMode.STOP);

            intakePivotServo.setPosition(0.0);

            retractHorizontalSlides();
        }

        // button is being held pivot intake and run intake motors
        if (gamepad2.x) {
            intakePivotServo.setPosition(1.0);
            runIntakeMotors(IntakeMode.NORMAL);

        }
        if (gamepad2.y) {
            intakePivotServo.setPosition(0.0);
            runIntakeMotors(IntakeMode.STOP);

            retractHorizontalSlides();
        }

        // intake slides have fully retracted, with threshold of 20 encoder ticks
        if (intakeSlidesMotor.getCurrentPosition() > -20) {
            // set isRetracting flag back to false
            isRetracting = false;

            // if intake is full, run intake motors to deposit block in basket
            if (intakeFull) {

                intakeFull = false;

                // start intaking block after 0.3 seconds
                new java.util.Timer().schedule(
                        new java.util.TimerTask() {
                            @Override
                            public void run() {
                                runIntakeMotors(IntakeMode.REVERSE);
                            }
                        },
                        300);

                // stop unloading block after 2 seconds
                new java.util.Timer().schedule(
                        new java.util.TimerTask() {
                            @Override
                            public void run() {
                                runIntakeMotors(IntakeMode.STOP);
                            }
                        },
                        2000);
            }
        }
    }

    public void horizontalSlides() {
        if (!isRetracting) {
            if (gamepad2.dpad_up && (intakeSlidesMotor.getCurrentPosition() >= -930)) { // max extension -930 ticks
                intakeSlidesMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                intakeSlidesMotor.setPower(-0.8);
            } else if (gamepad2.dpad_down && (intakeSlidesMotor.getCurrentPosition() <= 0)) {
                intakeSlidesMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                intakeSlidesMotor.setPower(0.8);
            } else {
                intakeSlidesMotor.setPower(0.0);
            }
        }
    }

    @Override
    public void stop() {
        setMotorPower(0, 0, 0, 0);
    }
}
