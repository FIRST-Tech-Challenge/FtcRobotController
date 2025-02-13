package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "twoFeetAutoBasketProgramV2")
public class twoFeetAutoBasketProgramV2 extends LinearOpMode {
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

    private static ElapsedTime forwardStopwatch = new ElapsedTime(); // timer for beginning forward setPower movement
    private static ElapsedTime liftBreakTimer = new ElapsedTime(); // lets lift go down before intake movements
    private static ElapsedTime intakeProcessTimer = new ElapsedTime();
    private boolean noHighBasket = true;
    private boolean noPickUp1 = true; // will use later
    private boolean isliftUp = false; // will use later
    private int i = 0;
    private boolean basket1stPos = false;
    private boolean basket2ndPos = false;
    private boolean reachedForwardIntake = false;
    private BNO055IMU imu;
    private DistanceSensor distanceSensor;

    private static final int targetRotationRight = 1680;
    private static final int targetRotationLeft = 600;
    private static final int forwardMovement = 325;
    private static final int forwardIntakeMovement = 200; // helps align robot to middle block
    private static final int secondForwardIntakeMovement = 75; // moves robot to middle block
    private static final int backwardMovement = 1600;
    private static final int leftwardMovement = 3100;

    public void basket() {
        if (basket1stPos == false) {
            basketServo.setPosition(1.0);
        }
        else if ((basketServo.getPosition() > 0.45) && (basket1stPos == false)) {
            basket1stPos = true;
        }
        sleep(3000);
        basketServo.setPosition(0.0);
        if (basketServo.getPosition() < 0.075) {
            basket2ndPos = true;
            basket1stPos = true;
        }
        sleep(3000);
    }

    public void rotateRight() {
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - targetRotationRight);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + targetRotationRight);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + targetRotationRight);
        backRight.setTargetPosition(backRight.getCurrentPosition() - targetRotationRight);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        frontLeft.setVelocity(1000);
        frontRight.setVelocity(1000);
        backLeft.setVelocity(1000);
        backRight.setVelocity(1000);
    }

    public void rotateLeft() {
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + targetRotationLeft);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - targetRotationLeft);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - targetRotationLeft);
        backRight.setTargetPosition(backRight.getCurrentPosition() + targetRotationLeft);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        frontLeft.setVelocity(1000);
        frontRight.setVelocity(1000);
        backLeft.setVelocity(1000);
        backRight.setVelocity(1000);
    }

    public void strafeLeft() {
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + leftwardMovement);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - leftwardMovement);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - leftwardMovement);
        backRight.setTargetPosition(backRight.getCurrentPosition() + leftwardMovement);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        frontLeft.setVelocity(1000);
        frontRight.setVelocity(1000);
        backLeft.setVelocity(1000);
        backRight.setVelocity(1000);
    }

    public void forward() {
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + forwardMovement);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + forwardMovement);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + forwardMovement);
        backRight.setTargetPosition(backRight.getCurrentPosition() + forwardMovement);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        frontLeft.setVelocity(1000);
        frontRight.setVelocity(1000);
        backLeft.setVelocity(1000);
        backRight.setVelocity(1000);

        if (frontLeft.getCurrentPosition() > 350) {
            reachedForwardIntake = true;
            resetMotors();
        }
    }

    public void intakeForward() {
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + forwardIntakeMovement);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + forwardIntakeMovement);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + forwardIntakeMovement);
        backRight.setTargetPosition(backRight.getCurrentPosition() + forwardIntakeMovement);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        frontLeft.setVelocity(1000);
        frontRight.setVelocity(1000);
        backLeft.setVelocity(1000);
        backRight.setVelocity(1000);

        // not using this right now
//        if (frontLeft.getCurrentPosition() > 5) {
//            telemetry.addData("Encoder pos for intake distance:", frontLeft.getCurrentPosition());
//            telemetry.update();
//            reachedForwardIntake = true;
//            resetMotors();
//        }
    }

    public void secondIntakeForward() {
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + secondForwardIntakeMovement);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + secondForwardIntakeMovement);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + secondForwardIntakeMovement);
        backRight.setTargetPosition(backRight.getCurrentPosition() + secondForwardIntakeMovement);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        frontLeft.setVelocity(1000);
        frontRight.setVelocity(1000);
        backLeft.setVelocity(1000);
        backRight.setVelocity(1000);

        // not using this right now
//        if (frontLeft.getCurrentPosition() > 5) {
//            telemetry.addData("Encoder pos for intake distance:", frontLeft.getCurrentPosition());
//            telemetry.update();
//            reachedForwardIntake = true;
//            resetMotors();
//        }
    }

    public void backward() {
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - backwardMovement);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - backwardMovement);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - backwardMovement);
        backRight.setTargetPosition(backRight.getCurrentPosition() - backwardMovement);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        frontLeft.setVelocity(1000);
        frontRight.setVelocity(1000);
        backLeft.setVelocity(1000);
        backRight.setVelocity(1000);
    }

    public void setLiftPosition(int targetPosition) {
        leftLiftMotor.setTargetPosition(-targetPosition);
        rightLiftMotor.setTargetPosition(targetPosition);
        leftLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftLiftMotor.setVelocity(1000);
        rightLiftMotor.setVelocity(1000);

        if (rightLiftMotor.getCurrentPosition() > 2890) {
            isliftUp = true;
        }
    }

    private void highBasketProcess() {
        // add back when basket is fixed
//        if (isliftUp == false) {
//            setLiftPosition(3266); // low basket encoder positions
//        }
//        else if ((isliftUp == true) && (basket2ndPos == false)) {
//            basket();
//        }
        basket2ndPos = true;
        basket1stPos = true;
        /* else */ if (basket2ndPos == true) { // add else back when basket is fixed
            i += 1;
            if (i == 1) {
                setLiftPosition(0);
                liftBreakTimer.reset();
                while (liftBreakTimer.seconds() < 4.0995) {
                }
                intakeProcess();

                stopMotors();
            }
        }
        else {
            noHighBasket = false;
        }
    }

    public enum IntakeMode {
        NORMAL,
        REVERSE,
        STOP
    }

    public void runIntakeMotors(twoFeetAutoBasketProgramV2.IntakeMode mode) {
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

    public void intakeProcess() {
        // will be used once we can pick up blocks
//        if (intakeFull == false) {

//            while (reachedForwardIntake == false) {

                intakeProcessTimer.reset();
                while (intakeProcessTimer.seconds() < .42) {
                    intakePivotServo.setPosition(1.0);
                    rotateLeft();
                    intakeForward(); // helps align robot to middle block
                }
                while (intakeProcessTimer.seconds() < .84) {
                    runIntakeMotors(twoFeetAutoBasketProgramV2.IntakeMode.NORMAL);
                    secondIntakeForward(); // moves robot to middle block
                }
//            }         // will be used once we can pick up blocks

            while (reachedForwardIntake == true) {
                intakePivotServo.setPosition(0.0);
                intakeFull = true;
                runIntakeMotors(twoFeetAutoBasketProgramV2.IntakeMode.STOP);
            }

        // will be used once we can pick up blocks
//            else if ((reachedForwardIntake == true) && (intakeFull == true) {
//                runIntakeMotors(twoFeetAutoBasketProgramV2.IntakeMode.REVERSE);
//
//            }

//            else if (reachedForwardIntake == true) {
//                intakePivotServo.setPosition(0.0);
//                intakeFull = true;
//            }
//        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the hardware
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        // Servo for basket
        basketServo = hardwareMap.get(Servo.class, "basketServo");

        // Servo for intake
        intakePivotServo = hardwareMap.get(Servo.class, "intakePivotServo");

        // Initialize lift motors
        leftLiftMotor = hardwareMap.get(DcMotorEx.class, "leftLiftMotor");
        rightLiftMotor = hardwareMap.get(DcMotorEx.class, "rightLiftMotor");

        // intake servos
        intakePivotServo = hardwareMap.get(Servo.class, "intakePivotServo");
        intakeServoLeft = hardwareMap.get(CRServo.class, "intakeServoLeft");
        intakeServoRight = hardwareMap.get(CRServo.class, "intakeServoRight");
        intakeTouchSensor = hardwareMap.get(TouchSensor.class, "intakeTouchSensor");

        // Wait for the start signal
        waitForStart();

        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        forwardStopwatch.reset();
        // robot travels 20 inches in 1 sec
        // goes 47.5 inches in 2 sec
        // goes 66 inches in 3 sec
        while (forwardStopwatch.seconds() < 0.35) {
            frontLeft.setPower(0.9);
            frontRight.setPower(0.9);
            backLeft.setPower(0.9);
            backRight.setPower(0.9);
            telemetry.addData("Status", "Strafing Right");
            telemetry.update();
        }
        strafeLeft();
        rotateRight();
        intakePivotServo.setPosition(-.2);
        while (noHighBasket) {
            highBasketProcess();
        }
        telemetry.addData("out of loop", rightLiftMotor.getCurrentPosition());
        telemetry.update();
    }

    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void resetMotors() {
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }
}
