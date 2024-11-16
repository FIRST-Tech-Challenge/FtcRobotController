package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "RobotStateMachine", group = "Iterative Opmode")
public class RobotStateMachineTeleOpIterative extends OpMode {

    // Drive motors
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    // Arm motor
    private DcMotor armMotor;

    // Wrist moto
    private DcMotor wristMotor;

    // Servos
    private Servo clawServo;
    private Servo intakeServo;

    // Arm and Wrist target positions for each state
    private static final int ARM_POSITION_INIT = 0;
    private static final int ARM_POSITION_INTAKE = 0;
    private static final int ARM_POSITION_WALL_GRAB = 1100;
    private static final int ARM_POSITION_WALL_UNHOOK = 1700;
    private static final int ARM_POSITION_HOVER_HIGH = 2600;
    private static final int ARM_POSITION_CLIP_HIGH = 2100;
    private static final int ARM_POSITION_LOW_BASKET = 2500;

    private static final int WRIST_POSITION_INIT = 0;
    private static final int WRIST_POSITION_SAMPLE = 270;
    private static final int WRIST_POSITION_SPEC = 10;

    // Claw positions
    private static final double CLAW_OPEN_POSITION = 0.55;
    private static final double CLAW_CLOSED_POSITION = 0.7;

    // Initial state
    private RobotState currentState = RobotState.INIT;

    // Claw toggle state
    private boolean clawOpen = false;
    private boolean lastBump = false;
    private boolean lastHook = false;
    private boolean lastGrab = false;

    //target position
    private int targetArm = 0;
    private int targetWrist = 0;

    @Override
    public void init() {
        // Initialize hardware components
        initHardware();

        // Telemetry for debugging
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Handle driving
        driveMecanum();

        // Handle wrist + arm movement
        controlArmAndWrist();

        // Handle claw operation
        controlClaw();

        // Handle intake system
        controlIntake();

        // Update telemetry data
        telemetry.update();
    }

    private void controlArmAndWrist() {
        // State machine logic
        switch (currentState) {
            case INIT:
                targetArm = ARM_POSITION_INIT;
                targetWrist = WRIST_POSITION_INIT;
                telemetry.addData("State", "INIT");
                break;

            case INTAKE:
                targetArm = ARM_POSITION_INTAKE;
                targetWrist = WRIST_POSITION_SAMPLE;
                telemetry.addData("State", "INTAKE");
                break;

            case WALL_GRAB:
                targetArm = ARM_POSITION_WALL_GRAB;
                targetWrist = WRIST_POSITION_SPEC;
                telemetry.addData("State", "WALL_GRAB");
                break;

            case WALL_UNHOOK:
                targetArm = ARM_POSITION_WALL_UNHOOK;
                targetWrist = WRIST_POSITION_SPEC;
                telemetry.addData("State", "WALL_UNHOOK");
                break;

            case HOVER_HIGH:
                targetArm = ARM_POSITION_HOVER_HIGH;
                targetWrist = WRIST_POSITION_SPEC;
                telemetry.addData("State", "HOVER_HIGH");
                break;

            case CLIP_HIGH:
                targetArm = ARM_POSITION_CLIP_HIGH;
                targetWrist = WRIST_POSITION_SPEC;
                telemetry.addData("State", "CLIP_HIGH");
                break;

            case LOW_BASKET:
                targetArm = ARM_POSITION_LOW_BASKET;
                targetWrist = WRIST_POSITION_SAMPLE;
                telemetry.addData("State", "LOW_BASKET");
                break;

            case MANUAL:
                telemetry.addData("State", "MANUAL");
                break;
        }

        // Handle state transitions based on gamepad input
        if (gamepad1.a) {
            currentState = RobotState.INTAKE;
        } else if (gamepad1.b && !lastGrab) {
            if (currentState == RobotState.WALL_GRAB) {
                currentState = RobotState.WALL_UNHOOK;
            } else {
                currentState = RobotState.WALL_GRAB;
            }
        } else if (gamepad1.y && !lastHook) {
            if (currentState == RobotState.HOVER_HIGH) {
                currentState = RobotState.CLIP_HIGH;
            } else {
                currentState = RobotState.HOVER_HIGH;
            }
        } else if (gamepad1.x) {
            currentState = RobotState.LOW_BASKET;
        } else if (gamepad1.left_bumper) {
            currentState = RobotState.INIT;
        } else if (gamepad1.dpad_up) { //manual control
            currentState = RobotState.MANUAL;
            targetArm += 10;
        } else if (gamepad1.dpad_down) {
            currentState = RobotState.MANUAL;
            targetArm -= 10;
        } else if (gamepad1.dpad_left) {
            currentState = RobotState.MANUAL;
            targetWrist += 1;
        } else if (gamepad1.dpad_right) {
            currentState = RobotState.MANUAL;
            targetWrist -= 1;
        }

        lastGrab = gamepad1.b;
        lastHook = gamepad1.y;

        armMotor.setTargetPosition(targetArm);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setTargetPosition(targetWrist);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        wristMotor.setPower(1);

        // Send telemetry data to the driver station
        telemetry.addData("Arm Position", armMotor.getCurrentPosition());
        telemetry.addData("Arm Power", armMotor.getPower());
        telemetry.addData("Wrist Position", wristMotor.getCurrentPosition());
        telemetry.addData("Wrist Power", wristMotor.getPower());
    }

    /**
     * Initialize all hardware components
     */
    private void initHardware() {
        // Initialize drive motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRight");

        // Initialize arm motor
        armMotor = hardwareMap.get(DcMotor.class, "arm");

        wristMotor = hardwareMap.get(DcMotor.class, "wrist");

        // Initialize servos
        clawServo = hardwareMap.get(Servo.class, "claw");
        intakeServo = hardwareMap.get(Servo.class, "intake");

        // Set motor directions (may need adjustment based on wiring)
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        stopAllMotors();

        // Set motors to run without encoders
        setWheelMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Arm and wrist mode
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set initial servo positions
        clawServo.setPosition(0.5);   // Neutral position
        intakeServo.setPosition(0.5); // Neutral position

        //Set zero power behavior
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Stop all motors
     */
    private void stopAllMotors() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        armMotor.setPower(0);
        wristMotor.setPower(0);
    }

    /**
     * Set motor modes for all drive motors
     */
    private void setWheelMotorModes(DcMotor.RunMode mode) {
        frontLeftMotor.setMode(mode);
        backLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
        backRightMotor.setMode(mode);
    }

    /**
     * Controls the robot's movement using mecanum drive calculations
     */
    private void driveMecanum() {
        // Retrieve joystick values
        double y = -gamepad1.left_stick_y; // Forward/backward (inverted)
        double x = gamepad1.left_stick_x * 1.1; // Strafing (adjusted for imperfect strafing)
        double rotation = gamepad1.right_stick_x; // Rotation

        double slowDownFactor = 0.6; // Factor to slow down robot movements

        // Calculate power for each wheel
        double frontLeftPower = y + x + rotation;
        double backLeftPower = y - x + rotation;
        double frontRightPower = y - x - rotation;
        double backRightPower = y + x - rotation;

        // Apply power to the wheels
        frontLeftMotor.setPower(frontLeftPower * slowDownFactor);
        backLeftMotor.setPower(backLeftPower * slowDownFactor);
        frontRightMotor.setPower(frontRightPower * slowDownFactor);
        backRightMotor.setPower(backRightPower * slowDownFactor);

        // Telemetry for debugging
        telemetry.addData("Front Left Power", frontLeftPower);
        telemetry.addData("Back Left Power", backLeftPower);
        telemetry.addData("Front Right Power", frontRightPower);
        telemetry.addData("Back Right Power", backRightPower);
    }

    /**
     * Controls the claw using buttons
     */
    private void controlClaw() {
        // Toggle claw position when right_bumper is pressed
        if (gamepad1.right_bumper && !lastBump) {
            clawOpen = !clawOpen;
            if (clawOpen) {
                clawServo.setPosition(CLAW_OPEN_POSITION);
            } else {
                clawServo.setPosition(CLAW_CLOSED_POSITION);
            }
        }
        lastBump = gamepad1.right_bumper;
        telemetry.addData("Claw Position", clawOpen ? "Open" : "Closed");
    }

    /**
     * Controls the intake system using buttons
     */
    private void controlIntake() {
        // Control intake servo with triggers
        if (gamepad1.right_trigger > 0.1) {
            intakeServo.setPosition(1.0);
        } else if (gamepad1.left_trigger > 0.1) {
            intakeServo.setPosition(0.0);
        } else {
            intakeServo.setPosition(0.5);
        }
    }
}