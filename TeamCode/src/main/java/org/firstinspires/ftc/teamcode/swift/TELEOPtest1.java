package org.firstinspires.ftc.teamcode.swift;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@TeleOp
public class TELEOPtest1 extends LinearOpMode {
    private Servo launcherServo = null;


    static final double INCREMENT = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_FWD = 1.0;     // Maximum FWD power applied to motor
    static final double MAX_REV = -1.0;     // Maximum REV power applied to motor


    @Override
    public void runOpMode() {

        waitForStart();

        launcherServo = hardwareMap.get(Servo.class, "launcherServo");

        while (opModeIsActive()) {
            // check to see if we need to move the servo.
            if (gamepad1.left_bumper) {
                // move to 0 degrees.
                launcherServo.setPosition(0.5);
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

            waitForStart();

            if (isStopRequested()) return;


                double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
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
            }


        //start Asher

        // Define class members
        DcMotor motor;
        double power = 0;
        boolean rampUp = true;



            // Connect to motor (Assume standard left wheel)
            // Change the text in quotes to match any motor name on your robot.
            motor = hardwareMap.get(DcMotor.class, "left_drive");

            // Wait for the start button
            telemetry.addData(">", "Press Start to run Motors.");
            telemetry.update();
            waitForStart();

            // Ramp motor speeds till stop pressed.
            while (opModeIsActive()) {

                // Ramp the motors, according to the rampUp variable.
                if (rampUp) {
                    // Keep stepping up until we hit the max value.
                    power += INCREMENT;
                    if (power >= MAX_FWD) {
                        power = MAX_FWD;
                        rampUp = !rampUp;   // Switch ramp direction
                    }
                } else {
                    // Keep stepping down until we hit the min value.
                    power -= INCREMENT;
                    if (power <= MAX_REV) {
                        power = MAX_REV;
                        rampUp = !rampUp;  // Switch ramp direction
                    }
                }

                // Display the current value
                telemetry.addData("Motor Power", "%5.2f", power);
                telemetry.addData(">", "Press Stop to end test.");
                telemetry.update();

                // Set the motor to the new power and pause;
                motor.setPower(power);
                sleep(CYCLE_MS);
                idle();
            }

            // Turn off motor and signal done;
            motor.setPower(0);
            telemetry.addData(">", "Done");
            telemetry.update();


            telemetry.update();
        }
    }

