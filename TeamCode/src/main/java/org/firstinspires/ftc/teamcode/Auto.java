package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Auto", group="Linear OpMode")
// @Disabled
public class Auto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftwheel = null;
    public DcMotor rightwheel = null;
    public Servo leftservo;
    public Servo rightservo;

    //    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
        leftwheel  = hardwareMap.get(DcMotor.class, "leftwheel");
        rightwheel = hardwareMap.get(DcMotor.class, "rightwheel");
        leftservo  = hardwareMap.get(Servo.class, "leftservo");
        rightservo = hardwareMap.get(Servo.class, "rightservo");

        // Set motor directions.
        leftwheel.setDirection(DcMotor.Direction.REVERSE);
        rightwheel.setDirection(DcMotor.Direction.FORWARD);

        // Set servo directions (keep default).
        leftservo.setDirection(Servo.Direction.FORWARD);
        rightservo.setDirection(Servo.Direction.FORWARD);


        // Wait for the game to start (driver presses START).
        waitForStart();
        runtime.reset();

        // Set the initial servo positions to neutral (mid-point).
        leftservo.setPosition(0.5);
        rightservo.setPosition(0.5);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup variables for motor power
            double leftPower;
            double rightPower;
            double radius;
            radius = 1;

            // Map the joystick inputs to motor power
            leftPower  = -gamepad1.left_stick_y;
            rightPower = -gamepad1.right_stick_y;
            leftPower = Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2));
            rightPower = Math.sqrt(Math.pow(gamepad1.right_stick_y, 2) + Math.pow(gamepad1.right_stick_x, 2));


            // Apply deadzone for joystick X-axis
            double leftStickX = gamepad1.left_stick_x;
            double rightStickX = gamepad1.right_stick_x;
            double deadzone = 0.05;

            // Servo control with deadzone
            if (Math.abs(leftStickX) > deadzone) {
                double leftServoPosition = (leftStickX + 1) / 2; // Map from -1 to 1 range to 0 to 1
                leftservo.setPosition(leftServoPosition);
            } else {
                leftservo.setPosition(0.5); // Neutral position
            }

            if (Math.abs(rightStickX) > deadzone) {
                double rightServoPosition = (rightStickX + 1) / 2; // Map from -1 to 1 range to 0 to 1
                rightservo.setPosition(rightServoPosition);
            } else {
                rightservo.setPosition(0.5); // Neutral position
            }

            // Send calculated power to wheels
            leftwheel.setPower(leftPower);
            rightwheel.setPower(rightPower);

            // Telemetry to display key data
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Left Motor Power", leftPower);
            telemetry.addData("Right Motor Power", rightPower);
            telemetry.addData("Left Servo Position", leftservo.getPosition());
            telemetry.addData("Right Servo Position", rightservo.getPosition());
            telemetry.addData("Left Stick X", leftStickX);
            telemetry.addData("Right Stick X", rightStickX);
            telemetry.update();
        }
    }
}
