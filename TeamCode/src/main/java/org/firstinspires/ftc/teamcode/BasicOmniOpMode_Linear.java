package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="Chocolate", group="Linear OpMode")

public class BasicOmniOpMode_Linear extends LinearOpMode {

    // Initialize all variables for the program
    private IMU imu = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
    double axial = 0;  // Note: pushing stick forward gives negative value
    double lateral = 0;
    double yaw = 0;

    double leftFrontPower = 0;
    double rightFrontPower = 0;
    double leftBackPower = 0;
    double rightBackPower = 0;

    private Servo leftArm = null;
    private Servo rightArm = null;
    private static final double ARM_DEFAULT = 0.5;
    private static final double ARM_MIN = 0.0;
    private static final double ARM_MAX = 1.0;

    private final ElapsedTime runtime = new ElapsedTime();

    // Variables for turning
    static final double TURN_SPEED_ADJUSTMENT = 0.015;     // Larger is more responsive, but also less stable
    static final double HEADING_ERROR_TOLERANCE = 1.0;    // How close must the heading get to the target before moving to next step.
    static final double MAX_TURN_SPEED = 1.0;     // Max Turn speed to limit turn rate
    static final double MIN_TURN_SPEED = 0.15;     // Min Turn speed to limit turn rate

    private double turnSpeed = 0;
    private double degreesToTurn = 0;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArm = hardwareMap.get(Servo.class, "left_arm");
        rightArm = hardwareMap.get(Servo.class, "right_arm");
        double arm_position = ARM_DEFAULT;
        leftArm.setPosition(arm_position);
        rightArm.setPosition(arm_position);

        // Initialize the IMU configuration
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            lateral = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            leftFrontPower = (axial + lateral + yaw) / 2;
            rightFrontPower = (axial - lateral - yaw) / 2;
            leftBackPower = (axial - lateral + yaw) / 2;
            rightBackPower = (axial + lateral - yaw) / 2;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // gyro turns
            if (gamepad1.dpad_up)
                turnToHeading(0.0);
            if (gamepad1.dpad_down)
                turnToHeading(180.0);
            if (gamepad1.dpad_left)
                turnToHeading(90);
            if (gamepad1.dpad_right)
                turnToHeading(-135.0); // Turn to face the basket

            if (gamepad1.left_bumper) {
                if (arm_position < ARM_MAX) {
                    arm_position += 0.001;
                    leftArm.setPosition(arm_position);
                }
            } else if (gamepad1.left_trigger > 0) {
                if (arm_position > ARM_MIN) {
                    arm_position -= 0.001;
                    leftArm.setPosition(ARM_MIN);
                }
            }

            if (gamepad1.right_bumper) {
                if (arm_position < ARM_MAX) {
                    arm_position += 0.001;
                    rightArm.setPosition(arm_position);
                }
            } else if (gamepad1.right_trigger > 0) {
                if (arm_position > ARM_MIN) {
                    arm_position -= 0.001;
                    rightArm.setPosition(ARM_MIN);
                }
            }
            // Show the elapsed game time and wheel power.
            telemetryData();
        }
    }

    private void telemetryData() {
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.addData("Joystick Axial", "%4.2f", axial);
        telemetry.addData("Joystick Lateral", "%4.2f", lateral);
        telemetry.addData("Joystick Yaw", "%4.2f", yaw);
        telemetry.addData("Current Yaw", "%.0f", getHeading());
        telemetry.addData("Turn Speed", "%4.2f", turnSpeed);
        telemetry.addData("Degrees to turn", "%4.2f", degreesToTurn);

        telemetry.update();
    }

    private void turnToHeading(double heading) {
        degreesToTurn = heading - getHeading();
        // Keep looping while we are still active, and not on heading.
        while (opModeIsActive()
                && (Math.abs(degreesToTurn) > HEADING_ERROR_TOLERANCE)
                && (gamepad1.left_stick_y == 0) && (gamepad1.left_stick_x == 0) && (gamepad1.right_stick_x == 0)) {

            degreesToTurn = heading - getHeading();
            if(degreesToTurn < -180) degreesToTurn += 360;
            if(degreesToTurn > 180) degreesToTurn -= 360;

            // Clip the speed to the maximum permitted value
            turnSpeed = Range.clip(degreesToTurn*TURN_SPEED_ADJUSTMENT, -MAX_TURN_SPEED, MAX_TURN_SPEED);
            if(turnSpeed < MIN_TURN_SPEED && turnSpeed >= 0) turnSpeed = MIN_TURN_SPEED;
            if(turnSpeed > -MIN_TURN_SPEED && turnSpeed < 0) turnSpeed = -MIN_TURN_SPEED;

            leftFrontDrive.setPower(-turnSpeed);
            rightFrontDrive.setPower(turnSpeed);
            leftBackDrive.setPower(-turnSpeed);
            rightBackDrive.setPower(turnSpeed);

            telemetryData();
        }
    }

    // read the Robot heading directly from the IMU (in degrees)
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}