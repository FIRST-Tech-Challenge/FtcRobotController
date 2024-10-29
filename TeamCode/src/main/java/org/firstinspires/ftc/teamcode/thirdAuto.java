package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.HashMap;

@Autonomous(name="thirdAuto", group="Auto")
public class thirdAuto extends OpMode {
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
        stateOne.armPosition = armPose.REST;
        stateOne.drivePose = new SparkFunOTOS.Pose2D(0, 0, 0);

        // State 2
        SystemState stateTwo = new SystemState();
        stateTwo.clawPosition = clawState.CLOSED;
        stateTwo.wristPosition = wristState.DOWN;
        stateTwo.armPosition = armPose.BASKET;
        stateTwo.drivePose = new SparkFunOTOS.Pose2D(0, 15, 0);

        // State 3
        SystemState stateThree = new SystemState();
        stateThree.clawPosition = clawState.CLOSED;
        stateThree.wristPosition = wristState.DOWN;
        stateThree.armPosition = armPose.BASKET;
        stateThree.drivePose = new SparkFunOTOS.Pose2D(22, 12, -135);

        // State 4
        SystemState stateFour = new SystemState();
        stateFour.clawPosition = clawState.CLOSED;
        stateFour.wristPosition = wristState.UP;
        stateFour.armPosition = armPose.BASKET;
        stateFour.drivePose = new SparkFunOTOS.Pose2D(22, 12, -135);

        // State 4A
        SystemState stateFourA = new SystemState();
        stateFourA.clawPosition = clawState.OPENED;
        stateFourA.wristPosition = wristState.UP;
        stateFourA.armPosition = armPose.BASKET;
        stateFourA.drivePose = new SparkFunOTOS.Pose2D(22, 12, -135);

        // State 5Pre1
        SystemState stateFivePre1 = new SystemState();
        stateFivePre1.clawPosition = clawState.OPENED;
        stateFivePre1.wristPosition = wristState.DOWN;
        stateFivePre1.armPosition = armPose.REST;
        stateFivePre1.drivePose = new SparkFunOTOS.Pose2D(12, 18, 0);

        // State 5
        SystemState stateFive = new SystemState();
        stateFive.clawPosition = clawState.OPENED;
        stateFive.wristPosition = wristState.DOWN;
        stateFive.armPosition = armPose.REST;
        stateFive.drivePose = new SparkFunOTOS.Pose2D(12, 24, 0);

        // State 5A
        SystemState stateFiveA = new SystemState();
        stateFiveA.clawPosition = clawState.CLOSED;
        stateFiveA.wristPosition = wristState.DOWN;
        stateFiveA.armPosition = armPose.REST;
        stateFiveA.drivePose = new SparkFunOTOS.Pose2D(12, 24, 0);

        // State 5B
        SystemState stateFiveB = new SystemState();
        stateFiveB.clawPosition = clawState.CLOSED;
        stateFiveB.wristPosition = wristState.DOWN;
        stateFiveB.armPosition = armPose.BASKET;
        stateFiveB.drivePose = new SparkFunOTOS.Pose2D(12, 24, 0);

        // State 6
        SystemState endState = new SystemState();
        endState.clawPosition = clawState.OPENED;
        endState.wristPosition = wristState.UP;
        endState.armPosition = armPose.ZERO;
        endState.drivePose = new SparkFunOTOS.Pose2D(0, 0, 0);

        // Generate Path
        HashMap<Integer, SystemState> instructions = new HashMap<>();
        instructions.put(0, stateOne); // Arm Up
        instructions.put(1, stateTwo); // Move forward
        instructions.put(2, stateThree); // Go to basket
        instructions.put(3, stateFour); // Wrist up
        instructions.put(4, stateFourA); // Claw Open
        instructions.put(5, stateFivePre1); // Go to pickup line 1
        instructions.put(6, stateFive); // Go to pickup line 1
//        instructions.put(7, stateFiveA); // Close Claw
//        instructions.put(8, stateFiveB); // Raise Arm
//        instructions.put(9, stateThree); // Go to basket
//        instructions.put(10, endState); // End at (0, 0)


        // ACS & DBS & Handler
        ArmSubSystem armControlSubsystem = new ArmSubSystem(armPose.ZERO, cap, spindle, lswitch, LSTop, LSLower, telemetry);
        DriveBaseSubsystem driveBaseSystem = new DriveBaseSubsystem(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, odometry, telemetry);
        autoHandler = new AutonomousHandler(instructions, armControlSubsystem, driveBaseSystem, 0, telemetry);
    }

    @Override
    public void loop() {
        autoHandler.periodicFunction();
    }
}
