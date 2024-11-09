package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;


@TeleOp(name="Swerve_drive", group="Linear OpMode")
// @Disabled
public class Swerve_drive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftwheel = null;
    public DcMotor rightwheel = null;
    public Servo leftservo;
    public Servo rightservo;
    public double rightpodposition;
    public double leftpodposition;
    public double leftpoddirection;
    public boolean rightpoddirection;
    public float leftwheelposition;
    public float rightwheelposition;

    //    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
        leftwheel = hardwareMap.get(DcMotor.class, "leftwheel");
        rightwheel = hardwareMap.get(DcMotor.class, "rightwheel");
        leftservo = hardwareMap.get(Servo.class, "leftservo");
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
            boolean leftMotorForward = true;
            boolean rightMotorForward = true;
            double leftPower;
            double rightPower;
            double radius;
            radius = 1;
            rightpodposition = 1;
            leftpodposition = 1;

            // Map the joystick inputs to motor power
            leftwheelposition = (gamepad1.left_stick_y);
            if (leftwheelposition < 0) {
                leftMotorForward = false;
            }

            if (rightwheelposition < 0) {
                rightMotorForward = false;
            }

            leftPower = Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2));
            rightPower = Math.sqrt(Math.pow(gamepad1.right_stick_y, 2) + Math.pow(gamepad1.right_stick_x, 2));


            // Apply deadzone for joystick X-axis
            double leftStickX = gamepad1.left_stick_x;
            double rightStickX = gamepad1.right_stick_x;
            double leftStickY = gamepad1.left_stick_y;
            double rightStickY = gamepad1.right_stick_y;
            double deadzone = 0;

            // left wheel

            leftpodposition = Math.toDegrees(Math.atan(gamepad1.left_stick_y / gamepad1.left_stick_x));
            double leftServoPosition = ((Math.abs(leftpodposition) / 180) + leftpoddirection); // Map from -1 to 1 range to 0 to 1
            rightpodposition = Math.toDegrees(Math.atan(gamepad1.right_stick_y / gamepad1.right_stick_x));
            double rightServoPosition = (Math.abs(rightpodposition) / 90); // Map from -1 to 1 range to 0 to 1

            // Quadrant 1

            if (leftStickY > 0 && rightStickX > 0) {
                if (Math.abs(leftStickX) > deadzone) {
                    leftservo.setPosition(leftServoPosition + 0.5);
                }
                if (gamepad1.left_bumper) {
                    leftservo.setPosition(0.5);
                }
                    // right wheel

                if (Math.abs(rightStickX) > deadzone) {
                    rightservo.setPosition(rightServoPosition + 0.5);
                }
                if (gamepad1.right_bumper) {
                    rightservo.setPosition(0.5);
                }
            }
            // quadrant 2

            if (leftStickY > 0 && rightStickX < 0);
                if (Math.abs(leftStickX) > deadzone) {
                    leftservo.setPosition(leftServoPosition);
                }
                if (gamepad1.left_bumper) {
                    leftservo.setPosition(0.5);
                }
                    // right wheel

                if (Math.abs(rightStickX) > deadzone) {
                    rightservo.setPosition(rightServoPosition);
                }
                if (gamepad1.right_bumper) {
                    rightservo.setPosition(0.5);
                }
            // quadrant 3

            if (leftStickY < 0 && rightStickX < 0);
                if (Math.abs(leftStickX) > deadzone) {
                    leftservo.setPosition(leftServoPosition + 0.5);
                }
                if (gamepad1.left_bumper) {
                    leftservo.setPosition(0.5);
                }
                    // right wheel

                if (Math.abs(rightStickX) > deadzone) {
                    rightservo.setPosition(rightServoPosition + 0.5);
                }
                if (gamepad1.right_bumper) {
                    rightservo.setPosition(0.5);
                }
            // quadrant 4

            if (leftStickY < 0 && rightStickX > 0);
                if (Math.abs(leftStickX) > deadzone) {
                    leftservo.setPosition(leftServoPosition);
                }

                if (gamepad1.left_bumper) {
                    leftservo.setPosition(0.5);
                }

                    // right wheel

                if (Math.abs(rightStickX) > deadzone) {
                    rightservo.setPosition(rightServoPosition);
                }

                if (gamepad1.right_bumper) {
                    rightservo.setPosition(0.5);
                }

//            } else {
//                leftservo.setPosition(0.5); // Neutral position
//                double leftServoPosition = (leftStickX + 1) / 2; // Map from -1 to 1 range to 0 to 1
//                leftservo.setPosition(leftServoPosition);
//            }

//            if (Math.abs(rightStickX) > deadzone) {
//                double rightServoPosition = (rightStickX + 1) / 2; // Map from -1 to 1 range to 0 to 1
//                rightservo.setPosition(rightServoPosition);
//            } else {
//                rightservo.setPosition(0.5); // Neutral position
//                double rightServoPosition = (rightStickX + 1) / 2; // Map from -1 to 1 range to 0 to 1
//                rightservo.setPosition(rightServoPosition);
                        }

                        // Send calculated power to wheels
//            if (leftMotorForward == true){
//                leftwheel.setPower(leftPower);
//            }else{
//                leftwheel.setPower(-leftPower);
//            }
//
//            if (rightMotorForward == true){
//                rightwheel.setPower(rightPower);
//            }else{
//                rightwheel.setPower(-rightPower);
                    }

                    // Telemetry to display key data
//                    telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Left Motor Power", leftPower);
//            telemetry.addData("Right Motor Power", rightPower);
//                    telemetry.addData("Left Servo Position", leftservo.getPosition());
//                    telemetry.addData("Right Servo Position", rightservo.getPosition());
//                    telemetry.addData("Left pod position", leftpodposition);
//            telemetry.addData("Left Stick X", leftStickX);
//            telemetry.addData("Right Stick X", rightStickX);
//                    telemetry.update();
                }