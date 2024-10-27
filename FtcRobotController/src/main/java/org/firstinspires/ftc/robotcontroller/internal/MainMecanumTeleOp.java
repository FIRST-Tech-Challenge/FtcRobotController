package org.firstinspires.ftc.robotcontroller.internal;

// Import necessary classes for FTC robot operation and hardware control
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

// Define the TeleOp mode with a name "Main Mecanum Drive"
@TeleOp(name = "Main Mecanum Drive")
public class MainMecanumTeleOp extends LinearOpMode {

    // Override the runOpMode method, which contains the main TeleOp loop code
    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize all the motors and servos with the corresponding hardware mappings
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor pivotMotor = hardwareMap.dcMotor.get("pivotMotor");
        DcMotor sliderMotor = hardwareMap.dcMotor.get("sliderMotor");
        CRServo pickUpServo = hardwareMap.crservo.get("intakeServo");
        Servo rotateIntake = hardwareMap.servo.get("rotateServo");

        // Reverse the right motors to make the mecanum wheel setup compatible with directional control
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset encoders for motors to have a clean start
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Configure motors to run without encoder feedback, except pivot and slider which need encoder-based position control
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize state variables for controls and flags
        int counterPivot = 1;          // Counter for pivot initialization
        boolean ifPivotUp = false;     // Track pivot state (up or down)
        boolean intakeReleased = false; // Track intake state (released or held)

        // Wait until the start button is pressed
        waitForStart();

        // Stop operation if the stop button is pressed
        if (isStopRequested()) return;

        // Main TeleOp control loop
        while (opModeIsActive()) {

            // Controller inputs for mecanum drive and other robot functionalities
            double y = gamepad1.left_stick_y;           // Forward/Backward control
            double x = -gamepad1.left_stick_x * 1.1;    // Strafe control with a slight scaling adjustment
            double rx = -gamepad1.right_stick_x;        // Rotation control

            // Controller buttons for additional controls
            boolean pivotMove = gamepad1.right_bumper;  // Trigger pivot movement
            double sliderUp = -gamepad1.right_trigger;  // Slider movement up
            double sliderDown = gamepad1.left_trigger;  // Slider movement down
            boolean pickDown = gamepad1.a;              // Intake pick-down control
            boolean pickUp = gamepad1.b;                // Intake pick-up control
            boolean pickStop = gamepad1.x;              // Stop intake movement
            boolean resetIntake = gamepad1.y;           // Reset intake position
            boolean turnLeft = gamepad1.dpad_left;      // Rotate intake left
            boolean turnRight = gamepad1.dpad_right;    // Rotate intake right
            boolean turnCenter = gamepad1.dpad_down;    // Center intake

            // Calculate motor power for mecanum drive using input values
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = ((y + x + rx) / denominator) / 1.5;
            double frontRightPower = ((y - x - rx) / denominator) / 1.5;
            double backLeftPower = ((y - x + rx) / denominator) / 1.5;
            double backRightPower = ((y + x - rx) / denominator) / 1.5;

            // Initial setup for slider motor (executed only once during the first loop iteration)
            if (counterPivot == 1) {
                rotateIntake.setPosition(0);  // Set intake to neutral position
            }

            // Control the slider movement based on triggers for precise adjustments
            if (sliderDown > 0) {
                sliderMotor.setPower(sliderDown);
            } else if (sliderUp < 0) {
                if (sliderMotor.getCurrentPosition() < -1400) {
                    sliderMotor.setPower(0);  // Stop the slider if it reaches maximum position
                }
                sliderMotor.setPower(sliderUp);
            }

            // Control intake and outtake mechanism using the servo motor
            if (pickUp) {
                pickUpServo.setPower(0.5);  // Pick-up operation
            } else if (pickDown) {
                pickUpServo.setPower(-0.5); // Pick-down operation
            } else if (pickStop) {
                pickUpServo.setPower(0);    // Stop intake rotation
            }

            // Rotate intake using D-Pad inputs
            if (turnLeft) {
                rotateIntake.setPosition(0.0);   // Rotate intake to left position
            } else if (turnRight) {
                rotateIntake.setPosition(1.0);   // Rotate intake to right position
            } else if (turnCenter) {
                rotateIntake.setPosition(0.85);  // Center the intake
            }

            // Control pivot motor for up and down movement
            if (pivotMove) {
                if (ifPivotUp) {
                    pivotMotor.setTargetPosition(-100);       // Move pivot to pick-up position
                    pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    pivotMotor.setPower(-0.5);
                    ifPivotUp = false;
                } else {
                    pivotMotor.setTargetPosition(-2700);      // Move pivot to drop-off position
                    pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    pivotMotor.setPower(0.5);
                    ifPivotUp = true;
                }
            }

            // Reset intake to initial or alternate position when button pressed
            if (resetIntake) {
                if (!intakeReleased) {
                    pivotMotor.setTargetPosition(-275);
                    pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    pivotMotor.setPower(-0.5);
                    rotateIntake.setPosition(0.85);   // Reset intake position
                    sliderMotor.setTargetPosition(-500);
                    sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sliderMotor.setPower(0.4);
                    intakeReleased = true;
                } else {
                    pivotMotor.setTargetPosition(-2700);
                    pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    pivotMotor.setPower(0.4);
                    rotateIntake.setPosition(0.0);   // Reset to neutral
                    sliderMotor.setTargetPosition(-200);
                    sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sliderMotor.setPower(0.4);
                    sliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    intakeReleased = false;
                }
            }

            // Set motor power for each wheel based on mecanum drive calculations
            frontLeftMotor.setPower(frontLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backLeftMotor.setPower(backLeftPower);
            backRightMotor.setPower(backRightPower);

            // Display telemetry data on the driver station for real-time monitoring
            telemetry.addLine("Centralized Telemetry:");
            telemetry.addData("Left Stick Y", y);
            telemetry.addData("Left Stick X", x);
            telemetry.addData("Right Stick X", rx);
            telemetry.addData("Right Trigger", sliderUp);
            telemetry.addData("Left Trigger", sliderDown);
            telemetry.addLine("Motor Encoders:");
            telemetry.addData("Front Left", frontLeftMotor.getCurrentPosition());
            telemetry.addData("Front Right", frontRightMotor.getCurrentPosition());
            telemetry.addData("Back Left", backLeftMotor.getCurrentPosition());
            telemetry.addData("Back Right", backRightMotor.getCurrentPosition());
            telemetry.addData("Pivot", pivotMotor.getCurrentPosition());
            telemetry.addData("Slider", sliderMotor.getCurrentPosition());
            telemetry.addLine("Servo Positions:");
            telemetry.addData("Rotate Intake", rotateIntake.getPosition());
            telemetry.addData("Intake Power", pickUpServo.getPower());
            telemetry.update();

            // Increment counter after first initialization
            counterPivot += 1;
        }
    }
}