package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Teleop_final", group="Linear OpMode")
// @Disabled
public class Teleop_final extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftwheel = null;
    public DcMotor rightwheel = null;
    public Servo leftservo;
    public Servo rightservo;
    public Servo grabber;
    public Servo tilt;
    public double tilt_position;
    public Servo arm;
    public double arm_position;

    //    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
        leftwheel  = hardwareMap.get(DcMotor.class, "leftwheel");
        rightwheel = hardwareMap.get(DcMotor.class, "rightwheel");
        leftservo  = hardwareMap.get(Servo.class, "leftservo");
        rightservo = hardwareMap.get(Servo.class, "rightservo");
        grabber  = hardwareMap.get(Servo.class, "grabber");
        tilt = hardwareMap.get(Servo.class, "tilt");
        arm = hardwareMap.get(Servo.class, "arm");

        // Set motor directions.
        leftwheel.setDirection(DcMotor.Direction.REVERSE);
        rightwheel.setDirection(DcMotor.Direction.FORWARD);

        // Set servo directions (keep default).
        leftservo.setDirection(Servo.Direction.FORWARD);
        rightservo.setDirection(Servo.Direction.FORWARD);
        grabber.setDirection(Servo.Direction.FORWARD);
        tilt.setDirection(Servo.Direction.FORWARD);
        arm.setDirection(Servo.Direction.FORWARD);


        // Wait for the game to start (driver presses START).
        waitForStart();
        runtime.reset();

        // Set the initial servo positions to neutral (mid-point).
        leftservo.setPosition(0.5);
        rightservo.setPosition(0.5);
        tilt.setPosition(0.5);
        grabber.setPosition(0.5);
        arm.setPosition(0.5);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup variables for motor power
            double leftPower;
            double rightPower;
            double radius;
            radius = 1;

            // Map the joystick inputs to motor power
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

            while (gamepad1.y) {
                tilt_position = tilt.getPosition();
                tilt.setPosition(tilt_position + 0.05);
            }

            while (gamepad1.a) {
                tilt_position = tilt.getPosition();
                tilt.setPosition(tilt_position - 0.05);
            }

            if (gamepad1.x) {
                grabber.setPosition(0);
            }

            if (gamepad1.b) {
                grabber.setPosition(1);
            }

            while (gamepad1.right_bumper) {
                arm_position = arm.getPosition();
                arm.setPosition(arm_position + 0.05);
            }

            while (gamepad1.right_bumper) {
                arm_position = arm.getPosition();
                arm.setPosition(arm_position - 0.05);
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
            telemetry.addData("tilt Servo Position", tilt.getPosition());
            telemetry.addData("grabber Servo Position", grabber.getPosition());
            telemetry.addData("Left Stick X", leftStickX);
            telemetry.addData("Right Stick X", rightStickX);
            telemetry.addData("arm Servo Position", arm.getPosition());
            telemetry.update();
        }
    }
}
