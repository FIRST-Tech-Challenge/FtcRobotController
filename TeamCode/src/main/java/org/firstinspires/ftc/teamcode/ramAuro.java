package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Autonomous(name="ramAuto", group="Auto")
public class ramAuro extends OpMode {
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

        InstructionHandler instructionHandler = new InstructionHandler();
        instructionHandler.addInstruction(
                5,
                new SparkFunOTOS.Pose2D(0,0,0),
                armPose.AUTO_1,
                wristState.UP,
                clawState.CLOSED
        );
        instructionHandler.addInstruction(
                5,
                new SparkFunOTOS.Pose2D(0,5,0),
                armPose.CHAMBER_A,
                wristState.UP,
                clawState.CLOSED
        );
        instructionHandler.addInstruction(
                5,
                new SparkFunOTOS.Pose2D(0,10,0),
                armPose.CHAMBER_A,
                wristState.UP,
                clawState.CLOSED
        );
        instructionHandler.addInstruction(
                5,
                new SparkFunOTOS.Pose2D(0,15,0),
                armPose.CHAMBER_A,
                wristState.UP,
                clawState.CLOSED
        );
        instructionHandler.addInstruction(
                5,
                new SparkFunOTOS.Pose2D(0,24.5,0),
                armPose.CHAMBER_A,
                wristState.UP,
                clawState.CLOSED
        );
        instructionHandler.addInstruction(
                5,
                new SparkFunOTOS.Pose2D(0,24.5,0),
                armPose.CHAMBER_A,
                wristState.UP,
                clawState.OPENED
        );
        instructionHandler.addInstruction(
                5,
                new SparkFunOTOS.Pose2D(0,10,0),
                armPose.REST,
                wristState.UP,
                clawState.OPENED
        );
        instructionHandler.addInstruction(
                5,
                new SparkFunOTOS.Pose2D(0,5,0),
                armPose.REST,
                wristState.UP,
                clawState.OPENED
        );
        instructionHandler.addInstruction(
                5,
                new SparkFunOTOS.Pose2D(-24,10,0),
                armPose.REST,
                wristState.UP,
                clawState.OPENED
        );
        instructionHandler.addInstruction(
                5,
                new SparkFunOTOS.Pose2D(-24,35,0),
                armPose.REST,
                wristState.UP,
                clawState.OPENED
        );
        instructionHandler.addInstruction(
                5,
                new SparkFunOTOS.Pose2D(-20,45,0),
                armPose.REST,
                wristState.UP,
                clawState.OPENED
        );
        instructionHandler.addInstruction(
                5,
                new SparkFunOTOS.Pose2D(-20,50,0),
                armPose.REST,
                wristState.UP,
                clawState.OPENED
        );
        instructionHandler.addInstruction(
                5,
                new SparkFunOTOS.Pose2D(-36,50,0),
                armPose.REST,
                wristState.UP,
                clawState.OPENED
        );
        instructionHandler.addInstruction(
                5,
                new SparkFunOTOS.Pose2D(-36,30,0),
                armPose.REST,
                wristState.UP,
                clawState.OPENED
        );
        instructionHandler.addInstruction(
                5,
                new SparkFunOTOS.Pose2D(-36,20,0),
                armPose.REST,
                wristState.UP,
                clawState.OPENED
        );
        instructionHandler.addInstruction(
                5,
                new SparkFunOTOS.Pose2D(-36,15,0),
                armPose.REST,
                wristState.UP,
                clawState.OPENED
        );
        instructionHandler.addInstruction(
                5,
                new SparkFunOTOS.Pose2D(-36,10,0),
                armPose.REST,
                wristState.UP,
                clawState.OPENED
        );
        instructionHandler.addInstruction(
                5,
                new SparkFunOTOS.Pose2D(-36,50,0),
                armPose.REST,
                wristState.UP,
                clawState.OPENED
        );
        instructionHandler.addInstruction(
                5,
                new SparkFunOTOS.Pose2D(-44,50,0),
                armPose.REST,
                wristState.UP,
                clawState.OPENED
        );
        instructionHandler.addInstruction(
                5,
                new SparkFunOTOS.Pose2D(-44,30,0),
                armPose.REST,
                wristState.UP,
                clawState.OPENED
        );
        instructionHandler.addInstruction(
                5,
                new SparkFunOTOS.Pose2D(-44,20,0),
                armPose.REST,
                wristState.UP,
                clawState.OPENED
        );
        instructionHandler.addInstruction(
                5,
                new SparkFunOTOS.Pose2D(-44,15,0),
                armPose.REST,
                wristState.UP,
                clawState.OPENED
        );
        instructionHandler.addInstruction(
                5,
                new SparkFunOTOS.Pose2D(-44,10,0),
                armPose.REST,
                wristState.UP,
                clawState.OPENED
        );
        // ACS & DBS & Handler
        ArmSubSystem armControlSubsystem = new ArmSubSystem(armPose.ZERO, cap, spindle, lswitch, LSTop, LSLower, telemetry);
        DriveBaseSubsystem driveBaseSystem = new DriveBaseSubsystem(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, odometry, telemetry);
        autoHandler = new AutonomousHandler(instructionHandler.getInstructions(), armControlSubsystem, driveBaseSystem, 0, telemetry);

        LSLower.setPosition(0.1);
        LSTop.setPosition(0.1);

    }

    @Override
    public void loop() {
        autoHandler.periodicFunction();
    }
}
