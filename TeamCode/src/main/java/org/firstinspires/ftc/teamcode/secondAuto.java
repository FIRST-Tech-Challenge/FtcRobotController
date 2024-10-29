package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.HashMap;

@Autonomous(name="secondAuto", group="Auto")
public class secondAuto extends OpMode {
    private AutonomousHandler autoHandler;
    private HashMap<Integer, SystemState> instructions;
    private ArmSubSystem armControlSubsystem;
    private DriveBaseSubsystem driveBaseSystem;

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor spindle;
    private SparkFunOTOS odometry;
    private DcMotor cap;
    // Servos
    private ServoImplEx LSLower;
    private ServoImplEx LSTop;
    private RevTouchSensor lswitch;

    @Override
    public void init() {
        // Motors & Sensors
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");
        DcMotor spindle = hardwareMap.get(DcMotor.class, "spindle");
        SparkFunOTOS odometry = hardwareMap.get(SparkFunOTOS.class, "odometry");
        DcMotor cap = hardwareMap.get(DcMotor.class, "cap");
        // Servos
        ServoImplEx LSLower = hardwareMap.get(ServoImplEx.class, "LSLower");
        ServoImplEx LSTop = hardwareMap.get(ServoImplEx.class, "LSTop");
        RevTouchSensor lswitch = hardwareMap.get(RevTouchSensor.class, "Lswitch");

        // Define Servo range
        LSLower.setPwmEnable();
        LSTop.setPwmEnable();
        LSLower.scaleRange(0, 1);
        LSTop.scaleRange(0, 1);

        // Set motor directions
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set motor modes
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Breaking mode
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cap.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // State 1
        SystemState stateOne = new SystemState();
        stateOne.clawPosition = clawState.CLOSED;
        stateOne.wristPosition = wristState.UP;
        stateOne.armPosition = armPose.CHAMBER_A;
        stateOne.drivePose = new SparkFunOTOS.Pose2D(12, 0, 0);

        // State Two
        SystemState stateTwo = new SystemState();
        stateTwo.clawPosition = clawState.CLOSED;
        stateTwo.wristPosition = wristState.UP;
        stateTwo.armPosition = armPose.BASKET;
        stateTwo.drivePose = new SparkFunOTOS.Pose2D(12, 0, 0);

        // State Three
        SystemState stateThree = new SystemState();
        stateThree.clawPosition = clawState.OPENED;
        stateThree.wristPosition = wristState.UP;
        stateThree.armPosition = armPose.BASKET;
        stateThree.drivePose = new SparkFunOTOS.Pose2D(12, 0, 0);

        // State Four
        SystemState stateFour = new SystemState();
        stateFour.clawPosition = clawState.OPENED;
        stateFour.wristPosition = wristState.UP;
        stateFour.armPosition = armPose.REST;
        stateFour.drivePose = new SparkFunOTOS.Pose2D(-50, 0, 0);

        // Generate Path
        instructions = new HashMap<>();
        instructions.put(0, stateOne);
        instructions.put(1, stateTwo);
        instructions.put(2, stateThree);
        instructions.put(3, stateFour);

        // ACS & DBS & Handler
        armControlSubsystem = new ArmSubSystem(armPose.ZERO, cap, spindle, lswitch, LSTop, LSLower, telemetry);
        driveBaseSystem = new DriveBaseSubsystem(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, odometry, telemetry);
        autoHandler = new AutonomousHandler(instructions, armControlSubsystem, driveBaseSystem, 0, telemetry);
    }

    @Override
    public void loop() {
        autoHandler.periodicFunction();
    }
}
