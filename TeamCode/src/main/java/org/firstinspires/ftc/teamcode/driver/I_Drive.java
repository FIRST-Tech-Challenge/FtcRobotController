package org.firstinspires.ftc.teamcode.driver;


import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import Auto.Left;


@TeleOp
public class I_Drive extends LinearOpMode {
    //Declaring Hardware
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor rightLeadScrew = null;
    private DcMotor leftLeadScrew = null;
    private DcMotor linearExtension = null;
    private Servo intakeWrist = null;
    private Servo bucketWrist = null;
    private CRServo intakeFingers = null;
    private Servo intakeArmLeft = null;
    private Servo intakeArmRight = null;
    private Servo rightClimb = null;
    private Servo leftClimb = null;
    private Servo planeServo = null;
    // Variables
    public static boolean DEBUG = false;
    private ElapsedTime runtime = new ElapsedTime();
    private final double pickupPosition = .65;
    private final double dropoffPosition = .2;
    private final double outSlide = .33;
    private final double inSlide = .82;
    private static final double SQUARE_SIZE = 23.0;
    public static final double FORWARD_SPEED = 0.4;
    public static final double TURN_SPEED = 0.4;
    private static final double MAX_SPEED = 1;
    public static double detectWait = 6.0;
    public static double PICKUP_POSITION = 0.2;
    public static double FRONTDROP_POSITION = 0.8; // Placeholder value, adjust as needed
    public static double DROPOFF_POSITION = 1;
    public static double PICKUP_POSITION2 = .8;
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
    private boolean intServoState = false;
    private boolean intServoReverse = false;


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
//Declaring Threads
    private Thread climbThread;
    private Thread intakeThread;
    private Thread driveThread;
    private Thread deliveryThread;
    private Thread spinThread;
    private Thread hangThread;
    private Thread planeThread;
//Wheel Numbers
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: goBilda Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4;    //3.778 ;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    ////////////////////////////////////////////////////////////////////////////////////////////////
    @Override
    public void runOpMode() throws InterruptedException {
        //Hardwaremap Hardware
        Servo intakeArmLeft = hardwareMap.servo.get("intakeArmLeft");
        Servo intakeArmRight = hardwareMap.servo.get("intakeArmRight");
        Servo intakeWrist = hardwareMap.servo.get("intakeWrist");
        Servo intakeFingers = hardwareMap.servo.get("intakeFingers");
        Servo bucketWrist = hardwareMap.servo.get("bucketWrist");
        Servo leftClimb = hardwareMap.servo.get("leftClimb");
        Servo rightClimb = hardwareMap.servo.get("rightClimb");
        Servo planeServo = hardwareMap.servo.get("planeServo");
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor rightLeadScrew = hardwareMap.dcMotor.get("rightLeadScrew");
        DcMotor leftLeadScrew = hardwareMap.dcMotor.get("leftLeadScrew");
        DcMotor linearExtension = hardwareMap.dcMotor.get("linearExtension");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rightLeadScrew.setDirection(DcMotor.Direction.REVERSE);
        leftLeadScrew.setDirection(DcMotor.Direction.REVERSE);
        linearExtension.setDirection(DcMotor.Direction.REVERSE);


        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLeadScrew.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLeadScrew.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initializeClimbThread();
        initializeIntakeThread();
        initializeDriveThread();
        initializeDeliveryThread();
        initializeSpinThread();
        initializeHangThread();
        initializePlaneThread();

        waitForStart();

        climbThread.start();
        intakeThread.start();
        driveThread.start();
        deliveryThread.start();
        spinThread.start();
        hangThread.start();
        planeThread.start();
        ////////////////////////////////////////////////////////////////////////////////////////////////
        if (climbThread != null) climbThread.interrupt();
        if (intakeThread != null) intakeThread.interrupt();
        if (driveThread != null) driveThread.interrupt();
        if (deliveryThread != null) deliveryThread.interrupt();
        if (spinThread != null) spinThread.interrupt();
        if (hangThread != null) hangThread.interrupt();
        if (planeThread != null) planeThread.interrupt();

        runtime.reset();
        sleep(3000);
        sleep(1000);

        telemetry.addData("Starting at", "%7d %7d %7d %7d",
                frontLeftMotor.getCurrentPosition(),
                backLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition(),
                backRightMotor.getCurrentPosition());
        rightLeadScrew.getCurrentPosition();
        leftLeadScrew.getCurrentPosition();
        linearExtension.getCurrentPosition();
        telemetry.update();


    }
    ////////////////////////////////////////////////////////////////////////////////////////////////
    private void initializeDriveThread() {
        ElapsedTime runtime = new ElapsedTime();
        driveThread = new Thread(new Runnable() {

            @Override
            public void run() {
                while (!Thread.interrupted()) {
                    double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
                    double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                    double rx = -gamepad1.right_stick_x;

                    // Denominator is the largest motor power (absolute value) or 1
                    // This ensures all the powers maintain the same ratio,
                    // but only if at least one is out of the range [-1, 1]
                    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                    double frontLeftPower = (y + x + rx) / denominator; //
                    double backLeftPower = (y - x + rx) / denominator;
                    double frontRightPower = (y - x - rx) / denominator;
                    double backRightPower = (y + x - rx) / denominator;

                    frontLeftMotor.setPower(frontLeftPower);
                    backLeftMotor.setPower(backLeftPower);
                    frontRightMotor.setPower(frontRightPower);
                    backRightMotor.setPower(backRightPower);
                }
            }
        });
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    private void initializeClimbThread() {
        ElapsedTime runtime = new ElapsedTime();
        climbThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!Thread.interrupted()) {
                    if(gamepad1.a) {
                        //Raise Climb
                    }
                    if(gamepad1.y) {
                        //Lower climb
                    }
                    if(gamepad2.b) {
                        //set line 3
                    }
                    if(gamepad2.y) {
                        //set line 2
                    }
                    if(gamepad2.x) {
                        //set line 1
                    }

                }
            }
        });
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    private void initializeIntakeThread() {
        ElapsedTime runtime = new ElapsedTime();
        intakeThread = new Thread(new Runnable() {

            @Override
            public void run() {
                while (!Thread.interrupted()) {
                    if (gamepad2.dpad_up) {
                        intakeArmLeft.setPosition(1);
                        intakeArmRight.setPosition(1);
                    }
                }
            }
        });
    }
    ///////////////////////////////////////////////////////////////////////////
    private void initializeSpinThread () {
        ElapsedTime runtime = new ElapsedTime();
        spinThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!Thread.interrupted()) {
                    if (gamepad2.dpad_right) {
                        intakeFingers.setPower(1);
                    }
                    if (gamepad2.dpad_left) {
                        intakeFingers.setPower(-1);
                    }
                }
            }
        });
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    private void initializeDeliveryThread() {
        ElapsedTime runtime = new ElapsedTime();
        deliveryThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!Thread.interrupted()){
                }
            }
        });
    }
    ///////////////////////////////////////////////////////////////////////////
    private void initializeHangThread() {
        ElapsedTime runtime = new ElapsedTime();
        hangThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!Thread.interrupted()){
                    if(gamepad2.dpad_down){

                    }
                }
            }
        });
    }
    //////////////////////////////////////////////////////////////////////////
    private void initializePlaneThread() {
        ElapsedTime runtime = new ElapsedTime();
        planeThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!Thread.interrupted()){
                    if(gamepad2.right_bumper){

                    }
                }
            }
        });
    }
}

