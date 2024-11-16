package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "AleenaManal", group = "Iterative Opmode")
public class AleenaManal extends OpMode {

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

    private double kP = 0.05;  // Proportional gain
    private double kI = 0.001; // Integral gain
    private double kD = 0.02; // Derivative gain
    private double kF = 0.0;  // Feedforward (optional)
    private double errorThreshold = 10; // Define a threshold in encoder ticks where we stop moving if we're within this range

    private double targetArmPosition = 0;  // The target position for the arm (in encoder ticks or other units)
    private double integral = 0;
    private double previousError = 0;
    private double previousTime = 0;  // Used to calculate deltaTime for derivative


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

        // Handle arm movement
        controlArm();

        //Handle wrist movement

        controlWrist();

        // Handle claw operation
        controlClaw();

        // Handle intake system
        controlIntake();

        // Update telemetry data
        telemetry.update();
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
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set initial servo positions
        clawServo.setPosition(0.5);   // Neutral position
        intakeServo.setPosition(0.5); // Neutral position
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
    private void setMotorModes(DcMotor.RunMode mode) {
        frontLeftMotor.setMode(mode);
        backLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
        backRightMotor.setMode(mode);
        armMotor.setMode(mode);
        wristMotor.setMode(mode);
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

    private void controlArm() {
        // Get the current position of the arm from the encoder
        double currentArmPosition = armMotor.getCurrentPosition();  // Example function to get the encoder position

        // --- Using the triggers to set the target position ---
        double armUp = gamepad1.right_trigger;  // Value between 0 and 1
        double armDown = gamepad1.left_trigger; // Value between 0 and 1

        // Set target position based on the triggers
        targetArmPosition = (armUp - armDown) * 1000; // Adjust the multiplier as needed

        // Calculate the error
        double error = targetArmPosition - currentArmPosition;

        // If the error is within the deadband, stop applying power
        if (Math.abs(error) < errorThreshold) {
            armMotor.setPower(0);  // Don't move if we're within the acceptable range
            return;  // Exit the loop early, no need to continue with PID adjustments
        }

        // PID control
        double integral = 0;
        double previousError = 0;
        double currentTime = System.nanoTime() / 1_000_000_000.0; // Time in seconds
        double deltaTime = currentTime - previousTime;
        previousTime = currentTime;

        integral += error * deltaTime;
        double derivative = (error - previousError) / deltaTime;

        double pidOutput = (kP * error) + (kI * integral) + (kD * derivative);

        // Limit the motor power to the range [-1, 1]
        double armPower = Range.clip(pidOutput, -1.0, 1.0);

        // Apply the power to the motor
        armMotor.setPower(armPower);

        // Telemetry for debugging
        telemetry.addData("Target Arm Position", targetArmPosition);
        telemetry.addData("Current Arm Position", currentArmPosition);
        telemetry.addData("Error", error);
        telemetry.addData("PID Output", pidOutput);
        telemetry.addData("Arm Power", armPower);
    }

    /**
     * Controls the arm movement using the triggers
     */
    private void controlWrist() {
        double wristPower = 0;

        // Control wrist motor with left bumper and right bumper
        if (gamepad1.right_bumper) {
            wristPower = 1.0; // Move wrist up
        } else if (gamepad1.left_bumper) {
            wristPower = -1.0; // Move wrist down
        }

        wristMotor.setPower(wristPower);
        telemetry.addData("Wrist Power", wristPower);
    }

    /**
     * Controls the claw using buttons
     */
    private void controlClaw() {
        if (gamepad1.a) {
            // Open claw
            clawServo.setPosition(1.0);
            telemetry.addData("Claw", "Opened");
        } else if (gamepad1.b) {
            // Close claw
            clawServo.setPosition(0.0);
            telemetry.addData("Claw", "Closed");
        }
    }

    /**
     * Controls the intake system using buttons
     */
    private void controlIntake() {
        if (gamepad1.x) {
            // Activate intake system
            intakeServo.setPosition(1.0);
            telemetry.addData("Intake System", "Activated");
        } else if (gamepad1.y) {
            // Deactivate intake system
            intakeServo.setPosition(0.0);
            telemetry.addData("Intake System", "Deactivated");
        } else {
            intakeServo.setPosition(0.5);
            telemetry.addData("Intake System", "Stop");
        }
    }
}