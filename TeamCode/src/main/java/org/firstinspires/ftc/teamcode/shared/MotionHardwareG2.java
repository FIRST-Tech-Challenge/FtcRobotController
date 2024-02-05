package org.firstinspires.ftc.teamcode.shared;


import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

public class MotionHardwareG2 {
    public GlobalConfig globalConfigG2 = null;

    public static boolean DEBUG = false;
    private LinearOpMode myOpMode = null;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor rightLeadScrew = null;
    private DcMotor leftLeadScrew = null;
    private DcMotor armMotor = null;
    private Servo wristServo = null;
    private Servo bucketServo = null;
    private CRServo intServo = null;
    private Servo leftInt = null;
    private Servo rightInt = null;
    //private Servo rightClimb = null;
    //private Servo leftClimb = null;
    //private Servo planeServo = null;

    // Variables

    private static final double SQUARE_SIZE = 23.0;
    public static final double FORWARD_SPEED = 0.4;
    public static final double TURN_SPEED = 0.4;
    private static final double MAX_SPEED = 1;
    public static double detectWait = 6.0;
    public static double PICKUP_POSITION = 0.2;
    public static double FRONTDROP_POSITION = 0.8; // Placeholder value, adjust as needed
    public static double DROPOFF_POSITION = 1;
    public static double PICKUP_POSITION2 = .8;

    // Gripper positions
    public static double LEFT_SERVO_OPEN = 0.35;
    public static double LEFT_SERVO_CLOSE = 0.0;
    public static double RIGHT_SERVO_OPEN = 0.25;
    public static double RIGHT_SERVO_CLOSE = 0.3;

    //arm encoder values
    public static int PICKUP_POSITION_ENCODER = 0;
    public static int DROPOFF_POSITION_ENCODER = -3630;

    //driving scales
    public static double driveScale = 1; //was .3
    public static double strafeScale = 1; // was .5
    public static double rotateScale = .3; // was .3

    // 180 turn constants
    public static double FAST_ROTATE_SPEED = 1.0;
    public static long TURN_180_TIME_MS = 333;
    private boolean isTurning180 = false;
    // Variable for slowmo state
    private boolean slowmoActive = false;
    private boolean slowmoToggle = false; // To track the toggle state

    private long turnStartTime = 0;


    private TelemetryPacket packet;
    public static double Launch_POSITION = 0.8;
    public static double HOLD_POSITION = 0.3;
    // Constants for the wider gripper open position
    public static double LEFT_SERVO_WIDE_OPEN = -0.4; // Adjust as needed
    public static double RIGHT_SERVO_WIDE_OPEN = 0.5;


    public enum Direction {
        RIGHT,
        LEFT
    }

    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: goBilda Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4;    //3.778 ;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    ////////////////////////////////////////////////////////////////////////////////////////////////
    public MotionHardwareG2(LinearOpMode opmode) {myOpMode = opmode;}
    public MotionHardwareG2(LinearOpMode opmode, GlobalConfig globalConfig) {
        myOpMode = opmode;
        this.globalConfigG2 = globalConfig;

    }


        ////////////////////////////////////////////////////////////////////////////////////////////////

    public void init() {

        frontLeftMotor = myOpMode.hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = myOpMode.hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = myOpMode.hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = myOpMode.hardwareMap.get(DcMotor.class, "backRightMotor");
        rightLeadScrew = myOpMode.hardwareMap.get(DcMotor.class, "rightLeadScrew");
        leftLeadScrew = myOpMode.hardwareMap.get(DcMotor.class, "leftLeadScrew");
        armMotor = myOpMode.hardwareMap.get(DcMotor.class, "armMotor");

        //frontLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        //frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        //backLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        //backRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rightLeadScrew.setDirection(DcMotor.Direction.REVERSE);
        leftLeadScrew.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.REVERSE);


        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLeadScrew.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLeadScrew.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLeadScrew.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLeadScrew.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLeadScrew.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLeadScrew.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(globalConfigG2.getActiveDeliveryMode() == GlobalConfig.AUTONOMOUS_DELIVERY_MODES.DROPPER) {
       }
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //TODO Once wrist/gripper is fixed move pixel load step to new function
        leftInt = myOpMode.hardwareMap.get(Servo.class, "leftInt");
        rightInt = myOpMode.hardwareMap.get(Servo.class, "rightInt");
        wristServo = myOpMode.hardwareMap.get(Servo.class, "wristServo");
        intServo = myOpMode.hardwareMap.get(CRServo.class, "intServo");
        bucketServo = myOpMode.hardwareMap.get(Servo.class, "bucketServo");
        //leftClimb = myOpMode.hardwareMap.get(Servo.class, "leftClimb");
        //rightClimb = myOpMode.hardwareMap.get(Servo.class, "rightClimb");
        //planeServo = myOpMode.hardwareMap.get(Servo.class, "planeServo");

        runtime.reset();
        sleep(3000);
        sleep(1000);

        myOpMode.telemetry.addData("Starting at",  "%7d %7d %7d %7d",
                frontLeftMotor.getCurrentPosition(),
                backLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition(),
                backRightMotor.getCurrentPosition());
                rightLeadScrew.getCurrentPosition();
                leftLeadScrew.getCurrentPosition();
                armMotor.getCurrentPosition();
        myOpMode.telemetry.update();


    }

    public void moveRobot(String path, double time) {

        switch (path.toLowerCase(Locale.ROOT)) {
            case "back":
                frontLeftMotor.setPower(FORWARD_SPEED);
                backLeftMotor.setPower(FORWARD_SPEED);
                frontRightMotor.setPower(FORWARD_SPEED);
                backRightMotor.setPower(FORWARD_SPEED);
                runtime.reset();
                while (myOpMode.opModeIsActive() && (runtime.seconds() < time)) {
                   // telemetry.addData("Path", "%s: %4.1f S Elapsed", path, runtime.seconds());
                    //telemetry.update();
                }

                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                break;
            default:
                frontLeftMotor.setPower(-FORWARD_SPEED);
                backLeftMotor.setPower(-FORWARD_SPEED);
                frontRightMotor.setPower(-FORWARD_SPEED);
                backRightMotor.setPower(-FORWARD_SPEED);
                runtime.reset();
                while (myOpMode.opModeIsActive() && (runtime.seconds() < time)) {
                    myOpMode.telemetry.addData("Path", "%s: %4.1f S Elapsed", path, runtime.seconds());
                    myOpMode.telemetry.update();
                }

                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
        }
    }

    //TODO Current we go 1 inch farther than we are telling the robot.
    //TODO The back right motor always goes farther than instructed.  Removing it from the isBusy check
    /**
     * Moves robot provided distance using motor encoders.  Reverse movement is achieved
     * by passing in a negative distance value.  Motion will stop if:
     * - The motors reach their target
     * - Timeout has been exceeded
     * - opMode is cancelled/stopped
     *
     * @param  speed    speed of the motor
     * @param  distance distance in inches you want the robot to move
     * @param  timeoutS failsafe time to stop motion if motors are still busy
     */
    public void moveRobot(double speed, double distance, double timeoutS) {

        //frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        int newFrontLeftTarget = frontLeftMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        int newBackLeftTarget = backLeftMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        int newFrontRightTarget = frontRightMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        int newBackRightTarget = backRightMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);

        frontLeftMotor.setTargetPosition(newFrontLeftTarget);
        backLeftMotor.setTargetPosition(newBackLeftTarget);
        frontRightMotor.setTargetPosition(newFrontRightTarget);
        backRightMotor.setTargetPosition(newBackRightTarget);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        frontRightMotor.setPower(Math.abs(speed));
        frontLeftMotor.setPower(Math.abs(speed));
        backRightMotor.setPower(Math.abs(speed));
        backLeftMotor.setPower(Math.abs(speed));

        while (myOpMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy())) {

            // Display it for the driver.
            myOpMode.telemetry.addData("Running to",  " %7d :%7d :%7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
            myOpMode.telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d",
                    frontLeftMotor.getCurrentPosition(), frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
            myOpMode.telemetry.update();
        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        myOpMode.telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d",
                frontLeftMotor.getCurrentPosition(), frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        myOpMode.telemetry.update();
        sleep(1000);
    }

    /////////////////////////////////////////////////lll////////////////////////////////////////////


    //Turn Right or Left 90 degrees
    //TODO Refactor to use new stopRobot() function
    //TODO Refactor to remove switch statement (redundant now that we are setting direction with ternary)
    public void turnRobot(Direction direction, double distance, double speed, double timeoutS) {

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        //frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        //backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        int newFrontLeftTarget = (direction == Direction.LEFT) ?
                frontLeftMotor.getCurrentPosition() + (int)(-distance * COUNTS_PER_INCH) :
                frontLeftMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        int newBackLeftTarget = (direction == Direction.LEFT) ?
                backLeftMotor.getCurrentPosition() + (int)(-distance * COUNTS_PER_INCH) :
                backLeftMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        int newFrontRightTarget = (direction == Direction.LEFT) ?
                frontRightMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH) :
                frontRightMotor.getCurrentPosition() + (int)(-distance * COUNTS_PER_INCH);
        int newBackRightTarget = (direction == Direction.LEFT) ?
                backRightMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH) :
                backRightMotor.getCurrentPosition() + (int)(-distance * COUNTS_PER_INCH);

        myOpMode.telemetry.addData("FL FR BL BR", "%7d :%7d :%7d :%7d",
                newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
        debugWait();

        frontLeftMotor.setTargetPosition(newFrontLeftTarget);
        backLeftMotor.setTargetPosition(newBackLeftTarget);
        frontRightMotor.setTargetPosition(newFrontRightTarget);
        backRightMotor.setTargetPosition(newBackRightTarget);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        frontRightMotor.setPower(Math.abs(speed));
        frontLeftMotor.setPower(Math.abs(speed));
        backRightMotor.setPower(Math.abs(speed));
        backLeftMotor.setPower(Math.abs(speed));

        switch (direction) {
            case LEFT:
                while (myOpMode.opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())) {

                    // Display it for the driver.
                    myOpMode.telemetry.addData("Running to",  " %7d :%7d :%7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                    myOpMode.telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d",
                            frontLeftMotor.getCurrentPosition(), frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
                    myOpMode.telemetry.update();
                }
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                break;
            case RIGHT:
                while (myOpMode.opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy())) {

                    // Display it for the driver.
                    myOpMode.telemetry.addData("Running to",  " %7d :%7d :%7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                    myOpMode.telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d",
                            frontLeftMotor.getCurrentPosition(), frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
                    myOpMode.telemetry.update();
                }
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                break;

        }
        debugWait();
    }

    // 1 = camera position
    // 2 = pixel drop position

   /* private void dropProp() {
        telemetry.addData("Dropping Pixel", "");
        telemetry.update();
        leftGripper.setPosition(0.9); // Adjust the position value as needed
        rightGripper.setPosition(0.1); // Adjust the position value as needed
        telemetry.addData("Pixel Dropped", "");
        telemetry.update();
        myOpMode.sleep(1000);
    }*/

    // Move along an arc to a grid position in inches relative to the robots current position
    //TODO This is an untested function
    public void moveToGridPosition(double xInches, double yInches, double speed) {
        // Calculate the angle to the target position in radians
        double angleRadians = Math.atan2(yInches, xInches);

        // Calculate the distance to the target position
        double distance = Math.hypot(xInches, yInches);

        // Calculate the target encoder counts based on distance
        int targetCounts = (int) (COUNTS_PER_INCH * distance);

        // Calculate the powers for each motor to achieve the specified angle
        double frontLeftPower = speed * Math.cos(angleRadians);
        double backLeftPower = speed * Math.sin(angleRadians);
        double frontRightPower = speed * Math.sin(angleRadians);
        double backRightPower = speed * Math.cos(angleRadians);

        // Normalize the powers so that the maximum is 1.0
        double maxPower = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))
        );

        frontLeftPower /= maxPower;
        backLeftPower /= maxPower;
        frontRightPower /= maxPower;
        backRightPower /= maxPower;

        // Set the target position for each motor
        int frontLeftTarget = frontLeftMotor.getCurrentPosition() + targetCounts;
        int backLeftTarget = backLeftMotor.getCurrentPosition() + targetCounts;
        int frontRightTarget = frontRightMotor.getCurrentPosition() + targetCounts;
        int backRightTarget = backRightMotor.getCurrentPosition() + targetCounts;

        // Set the motor targets and run to position mode
        frontLeftMotor.setTargetPosition(frontLeftTarget);
        backLeftMotor.setTargetPosition(backLeftTarget);
        frontRightMotor.setTargetPosition(frontRightTarget);
        backRightMotor.setTargetPosition(backRightTarget);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the motor powers
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        // Wait until all motors have reached their targets
        while (frontLeftMotor.isBusy() || backLeftMotor.isBusy() || frontRightMotor.isBusy() || backRightMotor.isBusy()) {
            // You can add additional logic here if needed
        }

        // Stop all motors
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        // Reset motor run mode to RUN_USING_ENCODER
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveToGridPosition(double xInches, double yInches) {
        moveToGridPosition(xInches, yInches, MAX_SPEED);
    }

    public void strafe(double distance, double speed, Direction direction, double timeoutS) {
        int newFrontLeftTarget = frontLeftMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        int newBackLeftTarget = backLeftMotor.getCurrentPosition() + (int)(-distance * COUNTS_PER_INCH);
        int newFrontRightTarget = frontRightMotor.getCurrentPosition() + (int)(-distance * COUNTS_PER_INCH);
        int newBackRightTarget = backRightMotor.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);

        frontLeftMotor.setTargetPosition(newFrontLeftTarget);
        backLeftMotor.setTargetPosition(newBackLeftTarget);
        frontRightMotor.setTargetPosition(newFrontRightTarget);
        backLeftMotor.setTargetPosition(newBackLeftTarget);

        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        runtime.reset();
        frontRightMotor.setPower(Math.abs(speed));
        frontLeftMotor.setPower(Math.abs(speed));
        backRightMotor.setPower(Math.abs(speed));
        backLeftMotor.setPower(Math.abs(speed));

        while (myOpMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy())) {

            // Display it for the driver.
            myOpMode.telemetry.addData("Running to",  " %7d :%7d :%7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
            myOpMode.telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d",
                    frontLeftMotor.getCurrentPosition(), frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
            myOpMode.telemetry.update();
        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        myOpMode.telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d",
                frontLeftMotor.getCurrentPosition(), frontRightMotor.getCurrentPosition(), backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition());
        myOpMode.telemetry.update();
        sleep(1000);
    }

    public void stopRobot() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////
    private void debugWait() {
        if (DEBUG) {
            sleep(5000);
        } else {
            sleep(1000);
        }
    }

}

    ////////////////////////////////////////////////////////////////////////////////////////////////





