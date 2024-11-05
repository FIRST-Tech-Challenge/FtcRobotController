package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.HashMap;

@Disabled
@Autonomous(name="fourthAuto", group="Auto")
public class fourthAuto extends OpMode {
    private AutonomousHandler autoHandler;

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
        LSLower.setPosition(0.1);
        LSTop.setPosition(0.1);

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
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        cap.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // State 1
        SystemState stateOne = new SystemState();
        stateOne.clawPosition = clawState.CLOSED;
        stateOne.wristPosition = wristState.DOWN;
        stateOne.armPosition = armPose.AUTO_1;
        stateOne.drivePose = new SparkFunOTOS.Pose2D(0, 0, 0);
        stateOne.ignorePosTime = 1;

        // State 1A
        SystemState stateOneA = new SystemState();
        stateOneA.clawPosition = clawState.CLOSED;
        stateOneA.wristPosition = wristState.DOWN;
        stateOneA.armPosition = armPose.BASKET_PREP;
        stateOneA.drivePose = new SparkFunOTOS.Pose2D(5, 9, -45);
        stateOneA.ignorePosTime = 2.5;

        // State 2
        SystemState stateTwo = new SystemState();
        stateTwo.clawPosition = clawState.CLOSED;
        stateTwo.wristPosition = wristState.UP;
        stateTwo.armPosition = armPose.BASKET;
        stateTwo.drivePose = new SparkFunOTOS.Pose2D(10, 9, -90);
        stateTwo.ignorePosTime = 3.5;

        // State 3
        SystemState stateThree = new SystemState();
        stateThree.clawPosition = clawState.CLOSED;
        stateThree.wristPosition = wristState.UP;
        stateThree.armPosition = armPose.BASKET;
        stateThree.drivePose = new SparkFunOTOS.Pose2D(30, 9, -135);
        stateThree.ignorePosTime = 2.5;

        // State 4
        SystemState stateFour = new SystemState();
        stateFour.clawPosition = clawState.CLOSED;
        stateFour.wristPosition = wristState.DOWN;
        stateFour.armPosition = armPose.BASKET;
        stateFour.drivePose = new SparkFunOTOS.Pose2D(30, 9, -135);
        stateFour.ignorePosTime = 0.5;

        // State 4A
        SystemState stateFourA = new SystemState();
        stateFourA.clawPosition = clawState.OPENED;
        stateFourA.wristPosition = wristState.UP;
        stateFourA.armPosition = armPose.BASKET;
        stateFourA.drivePose = new SparkFunOTOS.Pose2D(30, 9, -135);
        stateFourA.ignorePosTime = 0.5;

        // State 5PrePre2
        SystemState stateFivePrePre2 = new SystemState();
        stateFivePrePre2.clawPosition = clawState.OPENED;
        stateFivePrePre2.wristPosition = wristState.DOWN;
        stateFivePrePre2.armPosition = armPose.REST;
        stateFivePrePre2.drivePose = new SparkFunOTOS.Pose2D(15.75, 16, -85);
        stateFour.ignorePosTime = 2.5;

        // State 5PrePre1
        SystemState stateFivePrePre1 = new SystemState();
        stateFivePrePre1.clawPosition = clawState.OPENED;
        stateFivePrePre1.wristPosition = wristState.DOWN;
        stateFivePrePre1.armPosition = armPose.REST;
        stateFivePrePre1.drivePose = new SparkFunOTOS.Pose2D(15.75, 18, -45);
        stateFour.ignorePosTime = 2.5;

        // State 6
        SystemState endState = new SystemState();
        endState.clawPosition = clawState.OPENED;
        endState.wristPosition = wristState.UP;
        endState.armPosition = armPose.AUTO_1;
        endState.drivePose = new SparkFunOTOS.Pose2D(-72, 3, 0);
        endState.ignorePosTime = 30;

        // Generate Path
        HashMap<Integer, SystemState> instructions = new HashMap<>();
        instructions.put(0, stateOne); // Arm Up
        instructions.put(1, stateOneA); // Move Forward & Arm Up More
        instructions.put(2, stateTwo); // Move forward
        instructions.put(3, stateThree); // Go to basket
        instructions.put(4, stateFour); // Wrist up
        instructions.put(5, stateFourA); // Claw Open
        instructions.put(6, stateFivePrePre2); // PrePrep
        instructions.put(7, endState); // End at (0, 0)

        // ACS & DBS & Handler
        ArmSubSystem armControlSubsystem = new ArmSubSystem(armPose.ZERO, cap, spindle, lswitch, LSTop, LSLower, telemetry);
        DriveBaseSubsystem driveBaseSystem = new DriveBaseSubsystem(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, odometry, telemetry);
        autoHandler = new AutonomousHandler(instructions, armControlSubsystem, driveBaseSystem, 0, telemetry);

        LSLower.setPosition(0.1);
        LSTop.setPosition(0.1);

    }

    @Override
    public void loop() {
        autoHandler.periodicFunction();
    }
}
