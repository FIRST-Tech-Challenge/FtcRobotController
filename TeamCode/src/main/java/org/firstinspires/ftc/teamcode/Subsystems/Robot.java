package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.IOException;
import java.util.List;

/**
 * Created by AndrewC on 12/27/2019.
 */

public class Robot extends Subsystem {
    private boolean isBlue;
    public String name;
    private HardwareMap hardwareMap;
    private LinearOpMode opMode;
    public ElapsedTime timer;

    //DC Motors
    public DcMotorEx frontLeftDriveMotor;
    public DcMotorEx frontRightDriveMotor;
    public DcMotorEx rearRightDriveMotor;
    public DcMotorEx rearLeftDriveMotor;
    public DcMotorEx launch1;
    public DcMotorEx launch2a;
    public DcMotorEx launch2b;
    public DcMotorEx intake;

    //Servos

    public Servo elevatorR;
    public Servo elevatorL;

    public Servo wobbleClaw;
    public Servo wobbleGoalArm;
    public Servo intakeToElevatorL;
    public Servo intakeToElevatorR;
    public Servo launcherFeederL;
    public Servo launcherFeederR;

    //Odometry
    public List<LynxModule> allHubs;
    public DigitalChannel odometryRA;
    public DigitalChannel odometryRB;
    public DigitalChannel odometryBA;
    public DigitalChannel odometryBB;
    public DigitalChannel odometryLA;
    public DigitalChannel odometryLB;
    public int odRCount = 0;
    public int odBCount = 0;
    public int odLCount = 0;


    /**
     * Control Hub
     *
     * fr        0
     * br        1
     * launch2   2
     * launch1   3
     *
     * --------------------
     * Expansion Hub 2
     *
     * fl        0
     * bl        1
     * intake    2
     *
     * feeding          0
     * left elevator    1
     * wbc1             2
     * wbc2             3
     * right elevator   4
     */

    //Sensors
    public BNO055IMU imu;

    // Declare game pad objects
    public double leftStickX;
    public double leftStickY;
    public double rightStickX;
    public double rightStickY;
    public float triggerLeft;
    public float triggerRight;
    public boolean aButton = false;
    public boolean bButton = false;
    public boolean xButton = false;
    public boolean yButton = false;
    public boolean dPadUp = false;
    public boolean dPadDown = false;
    public boolean dPadLeft = false;
    public boolean dPadRight = false;
    public boolean bumperLeft = false;
    public boolean bumperRight = false;

    public double leftStickX2;
    public double leftStickY2;
    public double rightStickX2;
    public double rightStickY2;
    public float triggerLeft2;
    public float triggerRight2;
    public boolean aButton2 = false;
    public boolean bButton2 = false;
    public boolean xButton2 = false;
    public boolean yButton2 = false;
    public boolean dPadUp2 = false;
    public boolean dPadDown2 = false;
    public boolean dPadLeft2 = false;
    public boolean dPadRight2 = false;
    public boolean bumperLeft2 = false;
    public boolean bumperRight2 = false;

    public boolean isaButtonPressedPrev = false;
    public boolean isbButtonPressedPrev = false;
    public boolean isxButtonPressedPrev = false;
    public boolean isyButtonPressedPrev = false;
    public boolean isdPadUpPressedPrev = false;
    public boolean isdPadDownPressedPrev = false;
    public boolean isdPadLeftPressedPrev = false;
    public boolean isdPadRightPressedPrev = false;
    public boolean islBumperPressedPrev = false;
    public boolean isrBumperPressedPrev = false;
    public boolean isaButton2PressedPrev = false;
    public boolean isbButton2PressedPrev = false;
    public boolean isxButton2PressedPrev = false;
    public boolean isyButton2PressedPrev = false;
    public boolean isdPadUp2PressedPrev = false;
    public boolean isdPadDown2PressedPrev = false;
    public boolean isdPadLeft2PressedPrev = false;
    public boolean isdPadRight2PressedPrev = false;
    public boolean islBumper2PressedPrev = false;
    public boolean isrBumper2PressedPrev = false;

    private double joystickDeadZone = 0.1;

    //Subsystems
    public Drive drive;
    public Control control;
    public Vision vision;

    public Robot(LinearOpMode opMode, ElapsedTime timer) throws IOException {
        hardwareMap = opMode.hardwareMap;
        this.opMode = opMode;
        this.timer = timer;
        init();
    }

    /**
     *
     * @param opMode
     * @param timer
     * @param isBlue
     *          o: no camera is initialized
     *          1: only armWebcam is initialized for OpenCV
     *          2: backWebcam is initialized for Vuforia
     *          3: backWebcam is initialized for Vuforia and frontWebcam is initialized for OpenCV
     *          4: armWebcam is initialized for OpenCV and frontWebcam is initialized for OpenCV
     */
    public Robot(LinearOpMode opMode, ElapsedTime timer, boolean isBlue) throws IOException {
        hardwareMap = opMode.hardwareMap;
        this.opMode = opMode;
        this.timer = timer;
        if(isBlue) {
            this.isBlue = true;
        } else {
            this.isBlue = false;
        }
        init();
    }

    public void init() throws IOException {
        //DC Motors
        frontLeftDriveMotor = (DcMotorEx) hardwareMap.dcMotor.get("fl");
        frontRightDriveMotor = (DcMotorEx) hardwareMap.dcMotor.get("fr");
        rearLeftDriveMotor = (DcMotorEx) hardwareMap.dcMotor.get("bl");
        rearRightDriveMotor = (DcMotorEx) hardwareMap.dcMotor.get("br");

//        frontRightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        rearRightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftDriveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightDriveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rearLeftDriveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rearRightDriveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        launch1 = (DcMotorEx) hardwareMap.dcMotor.get("launch1");
        launch1.setDirection(DcMotorSimple.Direction.REVERSE);
        launch1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        launch1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launch1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        launch1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launch1.setPower(0.0);

        launch2a = (DcMotorEx) hardwareMap.dcMotor.get("launch2a");
        launch2a.setDirection(DcMotorSimple.Direction.REVERSE);
        launch2a.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        launch2a.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launch2a.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        launch2a.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launch2a.setPower(0.0);

        launch2b = (DcMotorEx) hardwareMap.dcMotor.get("launch2b");
        launch2b.setDirection(DcMotorSimple.Direction.REVERSE);
        launch2b.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        launch2b.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launch2b.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        launch2b.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launch2b.setPower(0.0);

        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setPower(0.0);

        //Servos
//        clawDeploy = hardwareMap.servo.get("clawDeploy");
//        claw = hardwareMap.servo.get("claw");
//        elevator1 = hardwareMap.crservo.get("e1");
//        elevator2 = hardwareMap.crservo.get("e2");
//
//        elevator2.setDirection(DcMotorSimple.Direction.REVERSE);
        wobbleClaw = hardwareMap.servo.get("wbc2");
        wobbleGoalArm = hardwareMap.servo.get("wbc1");
        launcherFeederR = hardwareMap.servo.get("feederR");
        launcherFeederL = hardwareMap.servo.get("feederL");
        intakeToElevatorR = hardwareMap.servo.get("iteR");
        intakeToElevatorL = hardwareMap.servo.get("iteL");


        elevatorR = hardwareMap.servo.get("elevatorR");
        elevatorL = hardwareMap.servo.get("elevatorL");

        allHubs = hardwareMap.getAll(LynxModule.class);

//        odometryRA  = hardwareMap.get(DigitalChannel.class, "odra");
//        odometryRB = hardwareMap.get(DigitalChannel.class, "odrb");
//        odometryBA  = hardwareMap.get(DigitalChannel.class, "odba");
//        odometryBB = hardwareMap.get(DigitalChannel.class, "odbb");
//        odometryLA  = hardwareMap.get(DigitalChannel.class, "odla");
//        odometryLB = hardwareMap.get(DigitalChannel.class, "odlb");
//
//        odometryRA.setMode(DigitalChannel.Mode.INPUT);          // Set the direction of each channel
//        odometryRB.setMode(DigitalChannel.Mode.INPUT);
//        odometryBA.setMode(DigitalChannel.Mode.INPUT);          // Set the direction of each channel
//        odometryBB.setMode(DigitalChannel.Mode.INPUT);
//        odometryLA.setMode(DigitalChannel.Mode.INPUT);          // Set the direction of each channel
//        odometryLB.setMode(DigitalChannel.Mode.INPUT);


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        opMode.telemetry.addData("Mode", " IMU initializing...");
        opMode.telemetry.update();
        imu.initialize(parameters);
        opMode.telemetry.addData("Mode", " IMU calibrating...");
        opMode.telemetry.update();
        // make sure the imu gyro is calibrated before continuing.
        while (opMode.opModeIsActive() && !imu.isGyroCalibrated())
        {
            opMode.sleep(50);
            opMode.idle();
        }

        //Subsystems
        opMode.telemetry.addData("Mode", " drive/control initializing...");
        opMode.telemetry.update();
        drive = new Drive(frontLeftDriveMotor, frontRightDriveMotor, rearLeftDriveMotor, rearRightDriveMotor, intake, launch1, launch2b, imu, opMode, timer);

//        drive.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        drive.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        control = new Control(intake, launch1, launch2, imu, opMode, timer, wobbleClaw, wobbleGoalArm);
//        control = new Control(intake, launch1, launch2a, launch2b, imu, opMode, timer, wobbleClaw, wobbleGoalArm);
        control = new Control(intake, launch1, launch2a, launch2b, imu, opMode, timer,
                wobbleClaw, wobbleGoalArm, intakeToElevatorR, intakeToElevatorL, launcherFeederR, launcherFeederL, elevatorR, elevatorL);


        opMode.telemetry.addData("Mode", " vision initializing...");
        opMode.telemetry.update();
        vision = new Vision(hardwareMap, this, isBlue);

    }

    public void initVisionTest() {
        vision = new Vision(hardwareMap, this, isBlue);
    }

    public void initServosAuto() {
        // code here
    }

    public void initServosTeleop() {
        control.moveWobbleGoalArmDown();
        control.closeWobbleGoalClaw();
    }

    public OpMode getOpmode(){
        return this.opMode;
    }

    public void getGamePadInputs() {
        isaButtonPressedPrev = aButton;
        isbButtonPressedPrev = bButton;
        isxButtonPressedPrev = xButton;
        isyButtonPressedPrev = yButton;
        isdPadUpPressedPrev = dPadUp;
        isdPadDownPressedPrev = dPadDown;
        isdPadLeftPressedPrev = dPadLeft;
        isdPadRightPressedPrev = dPadRight;
        islBumperPressedPrev = bumperLeft;
        isrBumperPressedPrev = bumperRight;
        leftStickX = joystickDeadzoneCorrection(opMode.gamepad1.left_stick_x);
        leftStickY = joystickDeadzoneCorrection(-opMode.gamepad1.left_stick_y);
        rightStickX = joystickDeadzoneCorrection(opMode.gamepad1.right_stick_x);
        rightStickY = joystickDeadzoneCorrection(opMode.gamepad1.right_stick_y);
        triggerLeft = opMode.gamepad1.left_trigger;
        triggerRight = opMode.gamepad1.right_trigger;
        aButton = opMode.gamepad1.a;
        bButton = opMode.gamepad1.b;
        xButton = opMode.gamepad1.x;
        yButton = opMode.gamepad1.y;
        dPadUp = opMode.gamepad1.dpad_up;
        dPadDown = opMode.gamepad1.dpad_down;
        dPadLeft = opMode.gamepad1.dpad_left;
        dPadRight = opMode.gamepad1.dpad_right;
        bumperLeft = opMode.gamepad1.left_bumper;
        bumperRight = opMode.gamepad1.right_bumper;

        isaButton2PressedPrev = aButton2;
        isbButton2PressedPrev = bButton2;
        isxButton2PressedPrev = xButton2;
        isyButton2PressedPrev = yButton2;
        isdPadUp2PressedPrev = dPadUp2;
        isdPadDown2PressedPrev = dPadDown2;
        isdPadLeft2PressedPrev = dPadLeft2;
        isdPadRight2PressedPrev = dPadRight2;
        islBumper2PressedPrev = bumperLeft2;
        isrBumper2PressedPrev = bumperRight2;
        leftStickX2 = joystickDeadzoneCorrection(opMode.gamepad2.left_stick_x);
        leftStickY2 = joystickDeadzoneCorrection(-opMode.gamepad2.left_stick_y);
        rightStickX2 = joystickDeadzoneCorrection(opMode.gamepad2.right_stick_x);
        rightStickY2 = joystickDeadzoneCorrection(-opMode.gamepad2.right_stick_y);
        triggerLeft2 = opMode.gamepad2.left_trigger;
        triggerRight2 = opMode.gamepad2.right_trigger;
        aButton2 = opMode.gamepad2.a;
        bButton2 = opMode.gamepad2.b;
        xButton2 = opMode.gamepad2.x;
        yButton2 = opMode.gamepad2.y;
        dPadUp2 = opMode.gamepad2.dpad_up;
        dPadDown2 = opMode.gamepad2.dpad_down;
        dPadLeft2 = opMode.gamepad2.dpad_left;
        dPadRight2 = opMode.gamepad2.dpad_right;
        bumperLeft2 = opMode.gamepad2.left_bumper;
        bumperRight2 = opMode.gamepad2.right_bumper;
    }

    public double joystickDeadzoneCorrection(double joystickInput) {
        double joystickOutput;
        if (joystickInput > joystickDeadZone) {
            joystickOutput = (joystickInput - joystickDeadZone) / (1.0-joystickDeadZone);
        }
        else if (joystickInput > -joystickDeadZone) {
            joystickOutput = 0.0;
        }
        else {
            joystickOutput = (joystickInput + joystickDeadZone) / (1.0-joystickDeadZone);
        }
        return joystickOutput;
    }


}