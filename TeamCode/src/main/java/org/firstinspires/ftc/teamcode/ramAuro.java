package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Autonomous(name="ramAuto", group="Auto")
public class ramAuro extends OpMode {
    private AutonomousHandler autoHandler;
    private ServoImplEx LSLower;
    private ServoImplEx LSTop;

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
        LSLower = hardwareMap.get(ServoImplEx.class, "LSLower");
        LSTop = hardwareMap.get(ServoImplEx.class, "LSTop");
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
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

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
                2.5,
                new SparkFunOTOS.Pose2D(0,0,0),
                armPose.AUTO_1,
                wristState.UP,
                clawState.CLOSED
        );
        // forward 10 arm up
        instructionHandler.addInstruction(
                1.5,
                new SparkFunOTOS.Pose2D(0,10,0),
                armPose.CHAMBER_A,
                wristState.UP,
                clawState.CLOSED
        );
        // forward 10 arm up
        instructionHandler.addInstruction(
                1.5,
                new SparkFunOTOS.Pose2D(0,10,0),
                armPose.CHAMBER_A,
                wristState.UP,
                clawState.CLOSED
        );
        // forward 27 arm up
        instructionHandler.addInstruction(
                2,
                new SparkFunOTOS.Pose2D(0,27,0),
                armPose.CHAMBER_A,
                wristState.UP,
                clawState.CLOSED
        );
        // forward arm lower
        instructionHandler.addInstruction(
                0.4,
                new SparkFunOTOS.Pose2D(0,32,0),
                armPose.CHAMBER_B,
                wristState.UP,
                clawState.CLOSED
        );
        // forward arm up open claw
        instructionHandler.addInstruction(
                0.5,
                new SparkFunOTOS.Pose2D(0,26,0),
                armPose.CHAMBER_A,
                wristState.UP,
                clawState.OPENED
        );
        //end of scoring first speciman, beginning scoring human player specimen, go right
        instructionHandler.addInstruction(
                1,
                new SparkFunOTOS.Pose2D(-12,10, 0),
                armPose.AUTO_PRE_2,
                wristState.UP,
                clawState.OPENED
        );
        // forward and right more
        instructionHandler.addInstruction(
                1,
                new SparkFunOTOS.Pose2D(-32,10, 0),
                armPose.AUTO_2,
                wristState.UP,
                clawState.OPENED
        );
        // forward and right and rotate right
        instructionHandler.addInstruction(
                2,
                new SparkFunOTOS.Pose2D(-44,50,45),
                armPose.AUTO_2,
                wristState.DOWN,
                clawState.OPENED
        );
        // moves back and left
        instructionHandler.addInstruction(
                1,
                new SparkFunOTOS.Pose2D(-33,50,0),
                armPose.AUTO_2,
                wristState.DOWN,
                clawState.OPENED
        );
        // back
        instructionHandler.addInstruction(
                2.25,
                new SparkFunOTOS.Pose2D(-33,18,0),
                armPose.AUTO_2,
                wristState.DOWN,
                clawState.OPENED
        );
        // forward
        instructionHandler.addInstruction(
                1,
                new SparkFunOTOS.Pose2D(-33,50,0),
                armPose.AUTO_2,
                wristState.DOWN,
                clawState.OPENED
        );
        // left
        instructionHandler.addInstruction(
                1.15,
                new SparkFunOTOS.Pose2D(-49,50,0),
                armPose.AUTO_2,
                wristState.DOWN,
                clawState.OPENED
        );
        // back
        instructionHandler.addInstruction(
                2,
                new SparkFunOTOS.Pose2D(-49,20,0),
                armPose.AUTO_2,
                wristState.DOWN,
                clawState.OPENED
        );
        // forward and left
        instructionHandler.addInstruction(
                0.3,
                new SparkFunOTOS.Pose2D(-30,40,0),
                armPose.AUTO_3,
                wristState.DOWN,
                clawState.OPENED
        );
        // moves back and turns 180 to pick up from human player
        instructionHandler.addInstruction(
                1.7,
                new SparkFunOTOS.Pose2D(-30,18,180),
                armPose.AUTO_3,
                wristState.DOWN,
                clawState.OPENED
        );
        instructionHandler.addInstruction(
                1,
                new SparkFunOTOS.Pose2D(-30,18,180),
                armPose.AUTO_3,
                wristState.DOWN,
                clawState.CLOSED
        );
        instructionHandler.addInstruction(
                1.5,
                new SparkFunOTOS.Pose2D(-30,18,180),
                armPose.AUTO_3,
                wristState.DOWN,
                clawState.CLOSED
        );
        instructionHandler.addInstruction(
                1.5,
                new SparkFunOTOS.Pose2D(12,20,0),
                armPose.CHAMBER_C,
                wristState.UP,
                clawState.CLOSED
        );
        instructionHandler.addInstruction(
                .75,
                new SparkFunOTOS.Pose2D(12,28.5,0),
                armPose.CHAMBER_C,
                wristState.UP,
                clawState.CLOSED
        );
        instructionHandler.addInstruction(
                .5,
                new SparkFunOTOS.Pose2D(12,28.5,0),
                armPose.CHAMBER_C,
                wristState.UP,
                clawState.CLOSED
        );
        instructionHandler.addInstruction(
                0.5,
                new SparkFunOTOS.Pose2D(12,34,0),
                armPose.CHAMBER_C,
                wristState.UP,
                clawState.CLOSED
        );
        instructionHandler.addInstruction(
                0.7,
                new SparkFunOTOS.Pose2D(12,30,0),
                armPose.CHAMBER_A,
                wristState.UP,
                clawState.CLOSED
        );
        instructionHandler.addInstruction(
                0.4,
                new SparkFunOTOS.Pose2D(12,28.5,0),
                armPose.CHAMBER_A,
                wristState.UP,
                clawState.OPENED
        );
        instructionHandler.addInstruction(
                0.75,
                new SparkFunOTOS.Pose2D(12,15,0),
                armPose.AUTO_2,
                wristState.UP,
                clawState.OPENED
        );
        instructionHandler.addInstruction(
                100,
                new SparkFunOTOS.Pose2D(-38,0,0),
                armPose.AUTO_FINISHER,
                wristState.UP,
                clawState.OPENED
        );
        // ACS & DBS & Handler
        ArmSubSystem armControlSubsystem = new ArmSubSystem(armPose.ZERO, cap, spindle, lswitch, LSLower, LSTop, telemetry);
        DriveBaseSubsystem driveBaseSystem = new DriveBaseSubsystem(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, odometry, telemetry);
        autoHandler = new AutonomousHandler(instructionHandler.getInstructions(), armControlSubsystem, driveBaseSystem, 0, telemetry);

        autoHandler.resetPID(0.03f, 0f, 0.015f);
        LSLower.setPosition(0.1);  // RC
        LSTop.setPosition(0.1);

    }

    @Override
    public void start() {
        super.start();
        LSLower.setPosition(0.1);  // RC
        LSTop.setPosition(0.1);
    }

    @Override
    public void loop() {
        autoHandler.periodicFunction();
    }
}
