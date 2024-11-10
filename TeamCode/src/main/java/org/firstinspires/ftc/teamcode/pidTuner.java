package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.HashMap;

@Config
@Autonomous(name="pidTuner", group="Auto")
public class pidTuner extends OpMode {
    private AutonomousHandler autoHandler;
    public static float p = 0.07F;
    public static float i = 0.000007f;
    public static float d = 0.001F;

    private float previous_p = p;
    private float previous_i = i;
    private float previous_d = d;

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
        stateOne.wristPosition = wristState.UP;
        stateOne.armPosition = armPose.REST;
        stateOne.drivePose = new SparkFunOTOS.Pose2D(0, 0, 0);
        stateOne.ignorePosTime = 100000;

        // State too
        SystemState stateToo = new SystemState();
        stateToo.clawPosition = clawState.CLOSED;
        stateToo.wristPosition = wristState.UP;
        stateToo.armPosition = armPose.REST;
        stateToo.drivePose = new SparkFunOTOS.Pose2D(95, 0, 0);
        stateToo.ignorePosTime = 100000;

        // State 2
        SystemState stateTwo = new SystemState();
        stateTwo.clawPosition = clawState.CLOSED;
        stateTwo.wristPosition = wristState.UP;
        stateTwo.armPosition = armPose.REST;
        stateTwo.drivePose = new SparkFunOTOS.Pose2D(95, 95, 0);
        stateTwo.ignorePosTime = 100000;

        // State 3 / tree
        SystemState statetree = new SystemState();
        statetree.clawPosition = clawState.CLOSED;
        statetree.wristPosition = wristState.UP;
        statetree.armPosition = armPose.REST;
        statetree.drivePose = new SparkFunOTOS.Pose2D(0, 95, 0);
        statetree.ignorePosTime = 100000;

        // Generate Path
        HashMap<Integer, SystemState> instructions = new HashMap<>();
        instructions.put(0, stateOne); // Arm Up
        instructions.put(1, stateToo); //lik ofrword or smth
        instructions.put(2, stateTwo); //lik frotneordward n stuf
        instructions.put(3, statetree); //lik backward n stuf
        instructions.put(4, stateOne); // Arm Up
        instructions.put(5, stateToo); //lik ofrword or smth
        instructions.put(6, stateTwo); //lik frotneordward n stuf
        instructions.put(7, statetree); //lik backward n stuf
        instructions.put(8, stateOne); // Arm Up
        instructions.put(9, stateToo); //lik ofrword or smth
        instructions.put(10, stateTwo); //lik frotneordward n stuf
        instructions.put(11, statetree); //lik backward n stuf
        instructions.put(12, stateOne); // Arm Up
        instructions.put(13, stateToo); //lik ofrword or smth
        instructions.put(14, stateTwo); //lik frotneordward n stuf
        instructions.put(15, statetree); //lik backward n stuf
        instructions.put(16, stateOne); // Arm Up
        instructions.put(17, stateToo); //lik ofrword or smth
        instructions.put(18, stateTwo); //lik frotneordward n stuf
        instructions.put(19, statetree); //lik backward n stuf
        instructions.put(20, stateOne); // Arm Up


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
        if ((p != previous_p) || (i != previous_i) || (d != previous_d)) {
            autoHandler.resetPID(p, i, d);
        }
        previous_p = p;
        previous_i = i;
        previous_d = d;
    }
}
