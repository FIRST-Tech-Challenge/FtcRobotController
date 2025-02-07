package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Main TeleOp mode for the robot.
 * Controls:
 * - DPad Up/Down: Manual intake forward/backward
 * - DPad Right: Retract intake
 * - DPad Left: Lower intake with flywheel forward
 * - Left Bumper: Lower intake with flywheel reverse
 * - Right Bumper: Move slide to low position and open claw
 * - Right Trigger: Close claw
 * - Left Trigger: Start scoring sequence
 * - Y Button: Move to ground and set elbow forward
 * - A Button: Move to ground and set elbow up
 */
@TeleOp(name = "IntoTheDeepTeleOp", group = "TeleOp")
public class IntoTheDeepTeleOp extends OpMode {
    // Robot subsystems
    private MecanumDrive mecanumDrive;
    private JoystickController joystickController;
    private Robot robot;
    private Intake intake;
    private Claw claw;
    private Flywheel flywheel;
    private ElapsedTime timer = new ElapsedTime();
    
    // Constants for vertical slide positions (encoder ticks)
    private static final int SLIDE_GROUND = 0;
    private static final int SLIDE_LOW = 800;
    private static final int SLIDE_MEDIUM = 1600;
    private static final int SLIDE_HIGH = 3000;
    
    // Constants for slide motor control
    private static final double MAX_POWER = 0.5;
    private static final double HOLDING_POWER = 0.1;  // Power to hold against gravity
    private static final int POSITION_TOLERANCE = 10; // Acceptable error in encoder ticks
    
    // Servo position constants
    private static final double CLAW_OPEN = 0.0;    // Claw servo open position
    private static final double CLAW_CLOSED = 1.0;  // Claw servo closed position
    private static final double WRIST_UP = 0.25;    // Wrist servo raised position
    private static final double WRIST_DOWN = 0.55;  // Wrist servo lowered position
    private static final double ELBOW_UP = 0.65;    // Elbow servo fully raised position
    private static final double ELBOW_FORWARD = 0.3; // Elbow servo horizontal position
    private static final double ELBOW_DOWN = 0.0;   // Elbow servo fully lowered position
    
    // State tracking variables
    private boolean isInScoringSequence = false;    // Tracks if scoring sequence is active
    private double scoringSequenceStartTime = 0;    // Time when scoring sequence started
    private static final double FLYWHEEL_RUN_TIME = 1; // Duration to run flywheel in seconds
    private boolean isMovingToLow = false;          // Tracks if slide is moving to low position
    
    // Hardware devices for manual control
    private DcMotorEx slideMotor;      // Vertical slide motor
    private TouchSensor limitSwitch;   // Bottom limit switch for slide
    private double currentSlidePower = 0.0;

    @Override
    public void init() {
        // Initialize drive motors
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "back_left");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "back_right");

        // Initialize subsystems
        mecanumDrive = new MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
        joystickController = new JoystickController(gamepad1, mecanumDrive);
        intake = new Intake(hardwareMap, "intake_slide");
        claw = new Claw(hardwareMap);
        flywheel = new Flywheel(hardwareMap, "flywheel");

        // Initialize and configure vertical slide motor and limit switch
        try {
            slideMotor = hardwareMap.get(DcMotorEx.class, "vertical_slide");
            limitSwitch = hardwareMap.get(TouchSensor.class, "limit_switch");
            
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor.setDirection(DcMotor.Direction.REVERSE);
            slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            
            telemetry.addData("Slide Status", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Slide Error", e.getMessage());
        }

        // Initialize robot controller
        robot = new Robot(mecanumDrive, joystickController);
        
        // Initial robot setup - move to starting position
        claw.moveToGround();  // Move slide to ground position
        claw.elbowUp();       // Raise elbow
        intake.in();          // Retract intake
        
        telemetry.addData("Status", "Initialized - Moving to ground position and retracting intake");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update drive controls from joystick input
        joystickController.update();
        
        // Manual intake motor control
        if (gamepad1.dpad_up) {
            intake.forward();      // Run intake motor forward
        } else if (gamepad1.dpad_down) {
            intake.backward();     // Run intake motor backward
        } else {
            intake.stop();         // Stop intake motor
        }
        
        // Intake position and flywheel control
        if (gamepad1.dpad_right) {
            intake.in();           // Retract intake
        } else if (gamepad1.dpad_left) {
            intake.down(true);     // Lower intake with flywheel forward
        } else if (gamepad1.left_bumper) {
            intake.down(false);    // Lower intake with flywheel reverse
        }
        
        // Slide to low position sequence
        if (gamepad1.right_bumper && !isMovingToLow) {
            claw.moveToLow();      // Start moving slide to low position
            isMovingToLow = true;
            telemetry.addData("Status", "Starting movement to low position...");
        }

        // Monitor and complete low position sequence
        if (isMovingToLow) {
            if (claw.isAtTargetPosition()) {
                claw.openClaw();   // Open claw when target position reached
                telemetry.addData("Status", "At target position, opening claw");
                isMovingToLow = false;
            } else {
                telemetry.addData("Status", "Moving to position... Current: %d", claw.getCurrentPosition());
            }
        } else {
            // Handle other claw controls when not moving to low position
            
            // Ground pickup position
            if (gamepad1.y) {
                claw.moveToGround();
                claw.elbowForward();
                claw.openClaw();
            }
            
            // Reset position
            if (gamepad1.a) {
                claw.moveToGround();
                claw.elbowUp();
            }
            
            // Claw grip control
            if (gamepad1.right_trigger > 0.1) {
                claw.closeClaw();
            }
            
            // Scoring sequence
            if (gamepad1.left_trigger > 0.1 && !isInScoringSequence) {
                isInScoringSequence = true;
                scoringSequenceStartTime = timer.seconds();
                // Initialize scoring position
                claw.moveToLow();
                claw.elbowDown();
                claw.wristDown();
                claw.openClaw();
                intake.in();
            }
        }
        
        // Handle scoring sequence timing
        if (isInScoringSequence) {
            double elapsedTime = timer.seconds() - scoringSequenceStartTime;
            
            if (claw.isAtTargetPosition()) {
                flywheel.start(false);  // Start flywheel when in position
                
                if (elapsedTime >= FLYWHEEL_RUN_TIME) {
                    // Complete scoring sequence
                    flywheel.stop();
                    claw.closeClaw();
                    claw.moveToHigh();
                    claw.elbowForward();
                    isInScoringSequence = false;
                }
            }
        }
        
        // Safety check for slide bottom limit
        claw.checkLimitSwitch();
        
        // Update debug information
        telemetry.addData("Slide Position", claw.getCurrentPosition());
        telemetry.addData("Slide Power", "%.2f", currentSlidePower);
        telemetry.addData("Limit Switch", limitSwitch.isPressed() ? "PRESSED" : "NOT PRESSED");
        telemetry.update();
    }
}

/* Example usage for both autonomous and teleop:
 * Robot myRobot = new Robot(new MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor),
 *                           new JoystickController(gamepad1, new MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor)));
 *
 * Teleop:
 * myRobot.updateJoystickControl();
 *
 * Autonomous:
 * myRobot.drive(0.5, 0, 0); // Move forward at 50% power
 */ 