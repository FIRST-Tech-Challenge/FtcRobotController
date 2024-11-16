package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Just Drive", group="Linear Opmode")
public class JustDriveOpMode extends LinearOpMode {

    // Define motors
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private Servo claw = null;
    private Servo intake = null;
    // Claw positions
    private static final double CLAW_OPEN_POSITION = -1.0;
    private static final double CLAW_CLOSED_POSITION = 1.0;


    @Override
    public void runOpMode() {
        // Initialize the hardware variables
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRight");

        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(Servo.class, "intake");

        // Set motor directions
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            // Get gamepad inputs
            double drive = -gamepad1.left_stick_y;  // Forward/backward
            double strafe = gamepad1.left_stick_x;  // Left/right
            double rotate = gamepad1.right_stick_x; // Rotation

            // Calculate power for each motor
            double frontLeftPower = drive + strafe + rotate;
            double frontRightPower = drive - strafe - rotate;
            double backLeftPower = drive - strafe + rotate;
            double backRightPower = drive + strafe - rotate;

            // Set power to motors
            frontLeftMotor.setPower(frontLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backLeftMotor.setPower(backLeftPower);
            backRightMotor.setPower(backRightPower);

            // Control the claw
            if (gamepad1.a) {
                closeClaw();
            } else if (gamepad1.b) {
                openClas();
            }

            // Control the intake
            if (gamepad1.left_trigger > 0.5) {
                startIntake();
            } else if (gamepad1.right_trigger > 0.5) {
                stopIntake();
            } else {
                stopIntake(); // Ensure it stops if neither trigger is pressed
            }

            // Add more controls or functionality as needed

            // Optional: Add telemetry for debugging
            telemetry.addData("Claw Position", claw.getPosition());
            telemetry.addData("Intake Power", intake.getPosition());
            telemetry.update();
        }
    }

    private void openClas() {
        claw.setPosition(CLAW_OPEN_POSITION); // Open claw
    }

    private void closeClaw() {
        claw.setPosition(CLAW_CLOSED_POSITION); // Close claw
    }

    private void startIntake() {
        intake.setPosition(1.0); // Start intake
    }

    private void stopIntake() {
        intake.setPosition(0.5); // Stop intake
    }
}
