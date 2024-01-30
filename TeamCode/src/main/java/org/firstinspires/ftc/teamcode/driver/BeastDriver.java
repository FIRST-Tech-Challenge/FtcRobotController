package org.firstinspires.ftc.teamcode.driver;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="BreeeastDrive", group="TeleOp Driver")
@Config
public class BeastDriver extends OpMode {

    // Drivetrain motors
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    // Arm motor
    private DcMotor armMotor;

    // Wrist and gripper servos
    private Servo wristServo, leftGripper, rightGripper;

    // FTC Dashboard
    private FtcDashboard dashboard;

    // Wrist positions
    public static double PICKUP_POSITION = 0.2;
    public static double FRONTDROP_POSITION = 0.8; // Placeholder value, adjust as needed
    public static double DROPOFF_POSITION = 1;
    public static double PICKUP_POSITION2 = .8;

    // Gripper positions
    public static double LEFT_SERVO_OPEN = 0.35;
    public static double LEFT_SERVO_CLOSE = 0.0;
    public static double RIGHT_SERVO_OPEN = 0.25;
    public static double RIGHT_SERVO_CLOSE = 0.3;

    //arm encoder values
    public static int PICKUP_POSITION_ENCODER = 0;
    public static int DROPOFF_POSITION_ENCODER = -3630;

    //driving scales
    public static double driveScale = 1; //was .3
    public static double strafeScale = 1; // was .5
    public static double rotateScale = .3; // was .3

    // 180 turn constants
    public static double FAST_ROTATE_SPEED = 1.0;
    public static long TURN_180_TIME_MS = 333;
    private boolean isTurning180 = false;
    // Variable for slowmo state
    private boolean slowmoActive = false;
    private boolean slowmoToggle = false; // To track the toggle state

    private long turnStartTime = 0;

    // Variables from planetest
    private Servo planeServo;
    private TelemetryPacket packet;
    public static double Launch_POSITION = 0.8;
    public static double HOLD_POSITION = 0.3;
    // Constants for the wider gripper open position
    public static double LEFT_SERVO_WIDE_OPEN = -0.4; // Adjust as needed
    public static double RIGHT_SERVO_WIDE_OPEN = 0.5; // Adjust as needed
    // Driving speed control


    @Override
    public void init() {
        // Drivetrain initialization
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        planeServo = hardwareMap.get(Servo.class, "planeServo");
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        //planeServo.setPosition(-0.2);
        // Arm initialization
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wrist and gripper initialization
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        leftGripper = hardwareMap.get(Servo.class, "leftGripper");
        rightGripper = hardwareMap.get(Servo.class, "rightGripper");

        // Initialize your servo in the hardware map
        //  planeServo = hardwareMap.servo.get("planeServo");
        //planeServo.setPosition(0);

        // FTC Dashboard initialization
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
    }

    @Override
    public void loop() {
        // plane logic
        if (gamepad2.dpad_up) {
            // Move planeServo to position - 1
            planeServo.setPosition(-0.2);
        } else {
            // Set it back to a default position when the button is not pressed
            // Replace 0.6 with your desired default position
            planeServo.setPosition(0.5);

            if (gamepad1.dpad_up && !isTurning180) {
                isTurning180 = true;
                turnStartTime = System.currentTimeMillis();

                frontLeftMotor.setPower(-FAST_ROTATE_SPEED);
                frontRightMotor.setPower(FAST_ROTATE_SPEED);
                backLeftMotor.setPower(-FAST_ROTATE_SPEED);
                backRightMotor.setPower(FAST_ROTATE_SPEED);
            }

            if (isTurning180 && (System.currentTimeMillis() - turnStartTime) > TURN_180_TIME_MS) {
                isTurning180 = false;

                frontLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
            }

            if (isTurning180) {
                return;
            }
            // Introduce separate scales for forward and backward driving
            double forwardDriveScale = 1; // Set this to your desired scale for forward motion (was.5)
            double backwardDriveScale = 1; // Set this to your desired scale for backward motion (was.3)

            // Toggle slowmo on dpad_up press
            if (gamepad1.right_stick_button && !slowmoToggle) {
                slowmoActive = !slowmoActive;
                slowmoToggle = true;
            } else if (!gamepad1.right_stick_button) {
                slowmoToggle = false;
            }

            double speedModifier = slowmoActive ? 0.25 : 1.0; // Reduce speed to 1/4 if slowmo is active
            // Drivetrain logic
            double rawDrive = -gamepad1.left_stick_y;
            double drive;
            if (rawDrive > 0) { // Forward
                drive = rawDrive * forwardDriveScale;
            } else { // Backward or no motion
                drive = rawDrive * backwardDriveScale;
            }

            double strafe = gamepad1.left_stick_x * strafeScale;
            double rotate = gamepad1.right_stick_x * rotateScale;

            double frontLeftPower = (drive + strafe + rotate) * speedModifier;
            double frontRightPower = (drive - strafe - rotate) * speedModifier;
            double backLeftPower = (drive - strafe + rotate) * speedModifier;
            double backRightPower = (drive + strafe - rotate) * speedModifier;


            double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            frontLeftMotor.setPower(frontLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backLeftMotor.setPower(backLeftPower);
            backRightMotor.setPower(backRightPower);

            // Arm logic
            // Arm logic
            if (gamepad1.dpad_down) {
                armMotor.setPower(1.0); // Negative power to move the arm down
            } else if (gamepad1.a && !gamepad1.dpad_down) {
                armMotor.setTargetPosition(PICKUP_POSITION_ENCODER);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1.0);
            } else if (gamepad1.y && !gamepad1.dpad_down) {
                armMotor.setTargetPosition(DROPOFF_POSITION_ENCODER);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1.0);
            } else {
                armMotor.setPower(0.0);
            }
            // Wrist logic
            if (gamepad1.left_bumper) {
                wristServo.setPosition(PICKUP_POSITION);
            } else if (gamepad1.right_bumper) {
                wristServo.setPosition(DROPOFF_POSITION);
            }

            // Gripper logic
            if (gamepad1.x) {
                leftGripper.setPosition(LEFT_SERVO_OPEN);
                rightGripper.setPosition(RIGHT_SERVO_OPEN);
            } else if (gamepad1.b) {
                leftGripper.setPosition(LEFT_SERVO_CLOSE);
                rightGripper.setPosition(RIGHT_SERVO_CLOSE);
            }

            // Resetting the armMotor Encoder
            if (gamepad1.dpad_left) {
                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (gamepad2.dpad_up) {
                // Move planeServo to position - 1
                planeServo.setPosition(-0.2);
            } else {
                // Set it back to a default position when the button is not pressed
                // Replace 0.6 with your desired default position
                planeServo.setPosition(0.5);
            }
            // Wrist servo control with triggers on gamepad2
            double wristPosition = wristServo.getPosition();
            double wristIncrement = 0.01; // Adjust this value for finer control

            if (gamepad2.right_trigger > 0) {
                // Right trigger on gamepad2 pressed - Move wrist up
                wristPosition += wristIncrement * gamepad2.right_trigger;
            } else if (gamepad2.left_trigger > 0) {
                // Left trigger on gamepad2 pressed - Move wrist down
                wristPosition -= wristIncrement * gamepad2.left_trigger;
            }

            // Ensure wrist position remains within valid range
            wristPosition = Range.clip(wristPosition, 0.0, 1.0);
            wristServo.setPosition(wristPosition);

            // Gripper logic for gamepad2 - wider open position
            if (gamepad2.b) {
                leftGripper.setPosition(LEFT_SERVO_WIDE_OPEN);
                rightGripper.setPosition(RIGHT_SERVO_WIDE_OPEN);
            }

            // Wrist logic
            if (gamepad2.left_bumper) {
                wristServo.setPosition(PICKUP_POSITION2);
            }

            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            if (gamepad2.x) {
                leftGripper.setPosition(LEFT_SERVO_OPEN);
                rightGripper.setPosition(RIGHT_SERVO_OPEN);
            }
            // Logic to set wrist position based on arm encoder value
            int armEncoderValue = armMotor.getCurrentPosition();
            if (armEncoderValue >= -500 && armEncoderValue <= 0) {
                wristServo.setPosition(DROPOFF_POSITION);
            } else if (armEncoderValue >= -1750 && armEncoderValue < -510) {
                wristServo.setPosition(FRONTDROP_POSITION);
            } else if (armEncoderValue < -1800) {
                wristServo.setPosition(PICKUP_POSITION);
            }





            // Telemetry
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Front Left Power", frontLeftMotor.getPower());
            packet.put("Front Right Power", frontRightMotor.getPower());
            packet.put("Back Left Power", backLeftMotor.getPower());
            packet.put("Back Right Power", backRightMotor.getPower());
            packet.put("Arm Power", armMotor.getPower());
            packet.put("Wrist Servo Position", wristServo.getPosition() == PICKUP_POSITION ? "Pickup" : "Dropoff");
            packet.put("Gripper", leftGripper.getPosition() == LEFT_SERVO_OPEN ? "Open" : "Closed");
            packet.put("Arm Encoder Value", armMotor.getCurrentPosition());
            dashboard.sendTelemetryPacket(packet);
        }
    }
}