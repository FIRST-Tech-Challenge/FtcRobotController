package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "twoFeetAutoBasketProgram")
public class twoFeetAutoBasketProgram extends LinearOpMode {
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx leftLiftMotor, rightLiftMotor;
    private Servo basketServo;
    private Servo intakePivotServo;

    private static ElapsedTime myStopwatch = new ElapsedTime();
    private static ElapsedTime basketTimer = new ElapsedTime();
    private boolean noHighBasket = true;
    private boolean isliftUp = false;
    private int i = 0;
    private boolean basket1stPos = false;
    private boolean basket2ndPos = false;

    private static final double wheelDiameter = 75;
    private static final double ticksPerRev = 100;
    private static final int targetRotation = 1550;
    private static final int forwardMovement = 400;
    private static final int backwardMovement = 400;
    private static final int leftwardMovement = 3100;
    private static final double BACK_WHEEL_RATIO = 2.0 / 3.0; // 3:2 sprocket ratio;

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
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - targetRotation);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + targetRotation);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + targetRotation);
        backRight.setTargetPosition(backRight.getCurrentPosition() - targetRotation);

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
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + leftwardMovement);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + leftwardMovement);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + leftwardMovement);
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

        if (rightLiftMotor.getCurrentPosition() > 3255) {
            isliftUp = true;
        }
    }

    private void highBasketProcess() {
        if (isliftUp == false) {
            setLiftPosition(3266); // low basket encoder positions
            telemetry.addData("Lift up", rightLiftMotor.getCurrentPosition());
            telemetry.update();
        }
        else if ((isliftUp == true) && (basket2ndPos == false)) {
            basket();
            telemetry.addData("Basket", rightLiftMotor.getCurrentPosition());
            telemetry.update();
        }
        else if (basket2ndPos == true) {
            i += 1;
            if (i == 1) {
                forward();
                stopMotors();
            }
            setLiftPosition(0); // low basket encoder positions
        }
        else {
            noHighBasket = false;
        }
    }

    public enum WallType {
        LEFT,
        BACK
    }

    /**
     * Continually call this function to align the robot to a wall.
     * @param wall: WallType.LEFT or WallType.BACK
     * @param distance: distance from wall in cm
     * @param angleOffset: angle offset from default position in degrees
     */
    public void alignToWall(WallType wall, double distance, double angleOffset) {
        if (wall == WallType.LEFT) {
            strafeLeft();
        } else if (wall == WallType.BACK) {
            backward();
        }
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

        myStopwatch.reset();
        // robot travels 20 inches in 1 sec
        // goes 47.5 inches in 2 sec
        // goes 66 inches in 3 sec
        while (myStopwatch.seconds() < 0.35) {
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
        stopMotors();
    }

    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}