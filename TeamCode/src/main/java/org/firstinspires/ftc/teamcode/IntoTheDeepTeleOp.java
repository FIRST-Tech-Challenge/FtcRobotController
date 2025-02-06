package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "IntoTheDeepTeleOp", group = "TeleOp")
public class IntoTheDeepTeleOp extends OpMode {
    private MecanumDrive mecanumDrive;
    private JoystickController joystickController;
    private Robot robot;
    private Intake intake;
    private Claw claw;
    private Flywheel flywheel;
    private ElapsedTime timer = new ElapsedTime();
    
    // Constants for slide positions
    private static final int SLIDE_PICKUP = 800;  // Between ground and low
    private static final int SLIDE_SCORING = 700; // Slightly below low
    private static final int SLIDE_GROUND = 0;
    private static final int SLIDE_LOW = 800;
    private static final int SLIDE_MEDIUM = 1600;
    private static final int SLIDE_HIGH = 3000;
    
    // Constants for slide control
    private static final double MAX_POWER = 0.5;
    private static final double HOLDING_POWER = 0.1;  // Power to hold against gravity
    private static final int POSITION_TOLERANCE = 10;
    
    // Servo positions
    private static final double CLAW_OPEN = 0.5;
    private static final double CLAW_CLOSED = 0.9;
    private static final double WRIST_UP = 0.25;
    private static final double WRIST_DOWN = 0.55;
    private static final double ELBOW_UP = 0.65;      // Fully raised position
    private static final double ELBOW_FORWARD = 0.3;  // Horizontal position
    private static final double ELBOW_DOWN = 0.0;     // Fully lowered position
    
    // State tracking for sequences
    private boolean isInScoringSequence = false;
    private double scoringSequenceStartTime = 0;
    private static final double FLYWHEEL_RUN_TIME = 0.5; // Time to run flywheel in seconds
    
    // Direct motor control for manual override
    private DcMotorEx slideMotor;
    private TouchSensor limitSwitch;
    private double currentSlidePower = 0.0;

    @Override
    public void init() {
        // Initialize hardware devices
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

        // Initialize direct motor control
        try {
            slideMotor = hardwareMap.get(DcMotorEx.class, "vertical_slide");
            limitSwitch = hardwareMap.get(TouchSensor.class, "limit_switch");
            
            // Configure motor
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor.setDirection(DcMotor.Direction.REVERSE);
            slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            
            telemetry.addData("Slide Status", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Slide Error", e.getMessage());
        }

        // Initialize the complete robot
        robot = new Robot(mecanumDrive, joystickController);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update joystick controls
        joystickController.update();
        
        // Intake controls
        if (gamepad1.dpad_up) {
            intake.forward();
        } else if (gamepad1.dpad_down) {
            intake.backward();
        } else {
            intake.stop();
        }
        
        // Existing intake controls
        if (gamepad1.dpad_right) {
            intake.in();
        } else if (gamepad1.dpad_left) {
            intake.down();
        } else if (gamepad1.left_bumper) {
            intake.down();
        }
        
        // Manual slide control with triggers when right bumper is held
        if (gamepad1.right_bumper) {
            // Direct slide control
            double upPower = -gamepad1.right_trigger * MAX_POWER;    // Negative = up
            double downPower = gamepad1.left_trigger * MAX_POWER;    // Positive = down
            currentSlidePower = upPower + downPower;
            
            // Check limit switch before applying downward power
            if (limitSwitch.isPressed() && currentSlidePower > 0) {
                currentSlidePower = 0;
                slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            
            // Apply power, using holding power when no input
            if (Math.abs(currentSlidePower) < 0.01) {
                slideMotor.setPower(HOLDING_POWER);
                currentSlidePower = HOLDING_POWER;
            } else {
                slideMotor.setPower(currentSlidePower);
            }
        } else {
            // Automatic control based on button presses
            // Y button - Low height and elbow forward
            if (gamepad1.y) {
                claw.moveToGround();
                claw.elbowForward();
                claw.openClaw();
            }
            
            // A button - Ground position and elbow up
            if (gamepad1.a) {
                claw.moveToGround();
                claw.elbowUp();
            }
            
            // B button - Pickup position and open claw
            if (gamepad1.b) {
                claw.closeClaw();
            }
            
            // Left trigger - Start scoring sequence
            if (gamepad1.left_trigger > 0.1 && !isInScoringSequence) {
                isInScoringSequence = true;
                scoringSequenceStartTime = timer.seconds();
                // Initial actions - move slide to low position first
                claw.moveToLow();
                claw.elbowDown();
                claw.wristDown();
                intake.in();
            }
        }
        
        // Handle scoring sequence
        if (isInScoringSequence) {
            double elapsedTime = timer.seconds() - scoringSequenceStartTime;
            
            // Wait for slide to reach target position before starting flywheel
            if (claw.isAtTargetPosition()) {
                // Only start flywheel after slide is in position
                flywheel.start(false); // Start flywheel
                
                if (elapsedTime >= FLYWHEEL_RUN_TIME) {
                    flywheel.stop(); // Stop flywheel
                    claw.closeClaw();
                    claw.moveToHigh();
                    claw.elbowForward();
                    isInScoringSequence = false;
                }
            }
        }
        
        // Always check limit switch
        claw.checkLimitSwitch();
        
        // Update telemetry
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