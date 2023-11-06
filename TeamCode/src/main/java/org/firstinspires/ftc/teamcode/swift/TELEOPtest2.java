package org.firstinspires.ftc.teamcode.swift;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class TELEOPtest2 extends LinearOpMode {
    private Servo launcherServo = null;


    static final double INCREMENT = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_FWD = 1.0;     // Maximum FWD power applied to motor
    static final double MAX_REV = -1.0;     // Maximum REV power applied to motor

    private Servo intakeServo;
    // Set the target position in encoder counts
    int upPosition = 650;
    int homePosition = 0;

    @Override
    public void runOpMode() {

        waitForStart();

        //Define variables for arm, wrist, gripper, and launcher
        Servo wristServo;
        DcMotor armMotor;
        DcMotor armMotor2;
        armMotor2 = hardwareMap.get(DcMotor.class, "armMotor2");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        wristServo = hardwareMap.servo.get("wristServo");

        launcherServo = hardwareMap.get(Servo.class, "launcherServo");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {

            // called when init button is  pressed.

            telemetry.addData("Mode", "waiting");



            // check to see if we need to move the servo.
            if (gamepad1.left_bumper) {
                // move to 0 degrees.
                launcherServo.setPosition(0.3);
            } else if (gamepad1.right_bumper || gamepad1.left_bumper) {
                // move to 90 degrees.
                launcherServo.setPosition(1);
            }
            telemetry.addData("Servo Position", launcherServo.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();


            // Declare our motors
            // Make sure your ID's match your configuration
            DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
            DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
            DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
            DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

            // Reverse the right side motors. This may be wrong for your setup.
            // If your robot moves backwards when commanded to go forwards,
            // reverse the left side instead.
            // See the note about this earlier on this page.
            frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


            if (isStopRequested()) return;


            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * -1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


            //start Asher


            // Run the motor to the target position
            if (gamepad2.right_trigger>.5) {
                armMotor.setTargetPosition(upPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.2); // Set the motor power (adjust as needed)

                armMotor2.setTargetPosition(upPosition);
                armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor2.setPower(0.2); // Set the motor power (adjust as needed)
            } else if (gamepad2.left_trigger>.5) {
                armMotor.setTargetPosition(homePosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.2); // Set the motor power (adjust as needed)

                armMotor2.setTargetPosition(homePosition);
                armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor2.setPower(0.2); // Set the motor power (adjust as needed)
            }

            while (armMotor.isBusy()) {
                // Wait for the motor to reach the target position
                telemetry.addData("Current Position", armMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop the motor after reaching the target position
            armMotor.setPower(0);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // check the gamepad buttons and if pressed, increment the appropriate position
            // variable to change the servo location.

            // move arm down on A button if not already at lowest position.

            intakeServo = hardwareMap.get(Servo.class, "intakeServo");

            if (gamepad2.y) {
                // Move servos in opposite directions when "y" is pressed
               // intakeServo.setPosition(0); // Adjust the position value as needed
                intakeServo.setDirection(Servo.Direction.FORWARD);

            } else if (gamepad2.x) {
                // Return servos to the center position when "x" is pressed
               // intakeServo.setPosition(1); // Adjust the position value for the center position
                intakeServo.setDirection(Servo.Direction.REVERSE);

            }
            if (gamepad2.b) {
                // Move servo in opposite directions when "y" is pressed
                wristServo.setPosition(.8 ); // Adjust the position value as needed

            } else if (gamepad2.a) {
                // Return servos to the center position when "x" is pressed
                wristServo.setPosition(.2); // Adjust the position value for the center position

                telemetry.update();
            }
        }
    }
}



