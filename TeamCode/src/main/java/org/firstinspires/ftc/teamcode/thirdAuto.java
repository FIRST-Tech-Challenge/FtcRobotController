package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
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
        instructionHandler.addInstruction( // Arm Up
                1,
                new SparkFunOTOS.Pose2D(0, 0, 0),
                armPose.AUTO_1,
                wristState.DOWN,
                clawState.CLOSED
        );
        int stateOneA = instructionHandler.addInstruction( // Move Forward & Arm Up More
                2.5,
                new SparkFunOTOS.Pose2D(5, 9, -45),
                armPose.BASKET_PREP,
                wristState.DOWN,
                clawState.CLOSED
        );
        int stateTwo = instructionHandler.addInstruction( // Move forward
                3.5,
                new SparkFunOTOS.Pose2D(10, 9, -90),
                armPose.BASKET,
                wristState.UP,
                clawState.CLOSED
        );
        int stateThree = instructionHandler.addInstruction( // Go to basket
                2.5,
                new SparkFunOTOS.Pose2D(30, 9, -135),
                armPose.BASKET,
                wristState.UP,
                clawState.CLOSED
        );
        int stateFour = instructionHandler.addInstruction( // Wrist up
                0.5,
                new SparkFunOTOS.Pose2D(30, 9, -135),
                armPose.BASKET,
                wristState.DOWN,
                clawState.CLOSED
        );
        int stateFourA = instructionHandler.addInstruction( // Claw Open
                0.5,
                new SparkFunOTOS.Pose2D(30, 9, -135),
                armPose.BASKET,
                wristState.UP,
                clawState.OPENED
        );
        instructionHandler.addInstruction( // PrePrep
                3.5,
                new SparkFunOTOS.Pose2D(15.5, 9, 0),
                armPose.REST,
                wristState.DOWN,
                clawState.OPENED
        );
        instructionHandler.addInstruction( // Prep pickup 1
                2.5,
                new SparkFunOTOS.Pose2D(15.5, 18, 0),
                armPose.REST,
                wristState.DOWN,
                clawState.OPENED
        );
        instructionHandler.addInstruction( // Prep pickup 2
                4,
                new SparkFunOTOS.Pose2D(15.5, 20, 0),
                armPose.REST,
                wristState.DOWN,
                clawState.CLOSED
        );
        instructionHandler.addInstruction( // Go to pickup line 1
                4,
                new SparkFunOTOS.Pose2D(15.75, 25, 0), // 27 Might work too
                armPose.PICKUP,
                wristState.DOWN,
                clawState.CLOSED
        );
        instructionHandler.addInstruction( // Close Claw
                4,
                new SparkFunOTOS.Pose2D(15.75, 27, 0),
                armPose.PICKUP,
                wristState.DOWN,
                clawState.CLOSED
        );
        instructionHandler.addInstruction( // Raise Arm
                4,
                new SparkFunOTOS.Pose2D(15.75, 27, 0),
                armPose.BASKET_PREP,
                wristState.DOWN,
                clawState.CLOSED
        );
        instructionHandler.reuseInstruction(stateOneA); // Move forward
        instructionHandler.reuseInstruction(stateTwo); // Move forward
        instructionHandler.reuseInstruction(stateThree); // Go to basket
        instructionHandler.reuseInstruction(stateFour); // Wrist up
        instructionHandler.reuseInstruction(stateFourA); // Claw Open
        instructionHandler.addInstruction( // Go to pickup line 2
                1,
                new SparkFunOTOS.Pose2D(27, 20, 0),
                armPose.REST,
                wristState.DOWN,
                clawState.OPENED
        );
        instructionHandler.addInstruction( // Go to pickup line 1
                1,
                new SparkFunOTOS.Pose2D(27, 24, 0),
                armPose.REST,
                wristState.DOWN,
                clawState.OPENED
        );
        instructionHandler.addInstruction( // Close Claw
                1,
                new SparkFunOTOS.Pose2D(27, 26, 0),
                armPose.PICKUP,
                wristState.DOWN,
                clawState.CLOSED
        );
        instructionHandler.addInstruction( // Raise Arm
                1,
                new SparkFunOTOS.Pose2D(27, 26, 0),
                armPose.BASKET,
                wristState.DOWN,
                clawState.CLOSED
        );
        instructionHandler.reuseInstruction(stateOneA); // Move forward
        instructionHandler.reuseInstruction(stateTwo); // Move forward
        instructionHandler.reuseInstruction(stateThree); // Go to basket
        instructionHandler.reuseInstruction(stateFour); // Wrist up
        instructionHandler.reuseInstruction(stateFourA); // Claw Open
        instructionHandler.addInstruction( // End at (0, 0)
                10,
                new SparkFunOTOS.Pose2D(0, 0, 0),
                armPose.ZERO,
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
