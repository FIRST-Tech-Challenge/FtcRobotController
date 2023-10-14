package org.firstinspires.ftc.teamcode.swift;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

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


        //Define variables for arm, wrist, and gripper
        Servo wristServo, left_Gripper, right_Gripper;
        DcMotor armMotor;
        //CRServo contServo;
        float leftY, rightY;
        double armPosition, gripPosition, contPower;
        double MIN_POSITION = 0, MAX_POSITION = 1;

        // called when init button is  pressed.

        {
            armMotor = hardwareMap.get(DcMotor.class, "armMotor");
            wristServo = hardwareMap.servo.get("Wrist Servo");
            left_Gripper = hardwareMap.servo.get("Left Gripper");
            right_Gripper = hardwareMap.servo.get("Right Gripper");
            //contServo = hardwareMap.crservo.get("cont_servo");

            telemetry.addData("Mode", "waiting");
            telemetry.update();

            // wait for start button.

            waitForStart();

            armPosition = .5;                   // set arm to half way up.
            gripPosition = MAX_POSITION;        // set grip to full open.

            while (opModeIsActive()) {
                leftY = gamepad1.left_stick_y * -1;
                rightY = gamepad1.right_stick_y * -1;

                armMotor.setPower(Range.clip(leftY, -1.0, 1.0));
                //rightMotor.setPower(Range.clip(rightY, -1.0, 1.0));

                telemetry.addData("Mode", "running");
                telemetry.addData("sticks", "  left=" + leftY + "  right=" + rightY);

                // check the gamepad buttons and if pressed, increment the appropriate position
                // variable to change the servo location.

                // move arm down on A button if not already at lowest position.
                if (gamepad1.a && armPosition > MIN_POSITION) armPosition -= .01;

                // move arm up on B button if not already at the highest position.
                if (gamepad1.b && armPosition < MAX_POSITION) armPosition += .01;

                // open the gripper on X button if not already at most open position.
                if (gamepad1.x && gripPosition < MAX_POSITION) gripPosition = gripPosition + .01;

                // close the gripper on Y button if not already at the closed position.
                if (gamepad1.y && gripPosition > MIN_POSITION) gripPosition = gripPosition - .01;

                // Set continuous servo power level and direction.
                if (gamepad1.dpad_left)
                    contPower = .20;
                else if (gamepad1.dpad_right)
                    contPower = -.20;
                else
                    contPower = 0.0;

                // set the servo position/power values as we have computed them.
                wristServo.setPosition(Range.clip(armPosition, MIN_POSITION, MAX_POSITION));
                left_Gripper.setPosition(Range.clip(gripPosition, MIN_POSITION, MAX_POSITION));
                right_Gripper.setPosition(Range.clip(gripPosition, MIN_POSITION, MAX_POSITION));
                // contServo.setPower(contPower);

                telemetry.addData("arm servo", "position=" + armPosition + "  actual=" + wristServo.getPosition());
                telemetry.addData("grip servo", "position=" + gripPosition + "    actual=" + left_Gripper.getPosition());
                telemetry.addData("grip servo", "position=" + gripPosition + "    actual=" + right_Gripper.getPosition());


                telemetry.update();
            }
        }
    }
}

