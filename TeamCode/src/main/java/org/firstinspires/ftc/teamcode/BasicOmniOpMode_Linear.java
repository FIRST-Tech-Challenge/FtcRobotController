package org.firstinspires.ftc.teamcode;

// All the things that we use and borrow

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="Remote Control", group="Linear OpMode")
public class BasicOmniOpMode_Linear extends LinearOpMode {
    // Initialize all variables for the program below:
    // This chunk controls our wheels
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    double leftFrontPower = 0;
    double rightFrontPower = 0;
    double leftBackPower = 0;
    double rightBackPower = 0;

    // Collect joystick position data
    double axial = 0;
    double lateral = 0;
    double yaw = 0;

    // This chunk controls our vertical
    private DcMotor vertical = null;
    private static final int VERTICAL_MAX = 650;
    private static final int VERTICAL_MIN = 30;
    private static final int VERTICAL_DEFAULT = 0;
    int verticalPosition = VERTICAL_DEFAULT;

    // This chunk controls our viper slide
    private DcMotor viperSlide = null;
    private static final int VIPER_MAX = 2600;
    private static final int VIPER_MIN = 50;
    private static final int VIPER_DEFAULT = 0;
    private int viperSlidePosition = VIPER_DEFAULT;

    // This chunk controls our claw
    private Servo claw = null;
    private static final double CLAW_DEFAULT = 0.75;
    private static final double CLAW_MIN = 0.96;
    private static final double CLAW_MAX = 0.75;
    double claw_position = CLAW_DEFAULT;

    private final ElapsedTime runtime = new ElapsedTime();

    // Variables for turns
    IMU imu = null;

    @Override
    //Op mode runs when the robot runs. It runs the whole time.
    public void runOpMode() {

        // Initialize the hardware variables.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        vertical = hardwareMap.get(DcMotor.class, "vertical");
        vertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertical.setTargetPosition(0);
        vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        viperSlide = hardwareMap.get(DcMotor.class, "viper_slide");
        viperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperSlide.setDirection(DcMotor.Direction.REVERSE);

        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(claw_position);

        // Initialize the IMU configuration
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // Get input from the joysticks
            axial = -gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power.
            leftFrontPower = (axial + lateral + yaw) / 2;
            rightFrontPower = (axial - lateral - yaw) / 2;
            leftBackPower = (axial - lateral + yaw) / 2;
            rightBackPower = (axial + lateral - yaw) / 2;

            // Normalize the values so no wheel power exceeds 100%
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

            // Control the vertical - the rotation level of the arm
            verticalPosition = vertical.getCurrentPosition();
            if (gamepad1.dpad_up) {
                vertical.setTargetPosition(VERTICAL_MAX);
                ((DcMotorEx) vertical).setVelocity(1000+viperSlidePosition/2.0);
                vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if (gamepad1.dpad_right && verticalPosition < VERTICAL_MAX) {          // If the right button is pressed AND it can safely raise further
                vertical.setTargetPosition(verticalPosition + 50);
                ((DcMotorEx) vertical).setVelocity(1000+viperSlidePosition/2.0);
                vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if (gamepad1.dpad_left && verticalPosition > VERTICAL_MIN) {           // If the left button is pressed AND it can safely lower further
                vertical.setTargetPosition(verticalPosition - 50);
                ((DcMotorEx) vertical).setVelocity(1000);
                vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if (gamepad1.dpad_down) {
                vertical.setTargetPosition(VERTICAL_MIN);
                ((DcMotorEx) vertical).setVelocity(1000);
                vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // Control the viper slide - how much it extends
            viperSlidePosition = viperSlide.getCurrentPosition();
            if (gamepad1.right_trigger > 0 && viperSlidePosition < VIPER_MAX) {              // If the right button is pressed AND it can safely extend further
                viperSlide.setTargetPosition(viperSlidePosition + 200);
                ((DcMotorEx) viperSlide).setVelocity(gamepad1.right_trigger*4000);
                viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if (gamepad1.left_trigger > 0 && viperSlidePosition > VIPER_MIN) {          // If the right button is pressed AND it can safely retract further
                viperSlide.setTargetPosition(viperSlidePosition - 200);
                ((DcMotorEx) viperSlide).setVelocity(gamepad1.left_trigger*4000);
                viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // Control the claw
            if (gamepad1.left_bumper && claw_position > CLAW_MAX) {
                claw_position -= 0.01;
            }
            if (gamepad1.right_bumper && claw_position < CLAW_MIN) {
                claw_position += 0.01;
            }
            claw.setPosition(claw_position);

            // Show the elapsed game time and wheel power.
            printDataOnScreen();
        }
    }

    // Log all (relevant) info about the robot on the hub.
    private void printDataOnScreen() {
        telemetry.addData("Run Time", "%.1f", runtime.seconds());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.addData("Joystick Axial", "%4.2f", axial);
        telemetry.addData("Joystick Lateral", "%4.2f", lateral);
        telemetry.addData("Joystick Yaw", "%4.2f", yaw);
        telemetry.addData("Current Yaw", "%.0f", getHeading());
        telemetry.addData("Claw position", "%4.2f", claw_position);
        telemetry.addData("Viper Slide Velocity", "%4.2f", ((DcMotorEx) viperSlide).getVelocity());
        telemetry.addData("Viper power consumption", "%.1f", ((DcMotorEx) viperSlide).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Viper Slide Position", "%d", viperSlidePosition);
        telemetry.addData("Vertical Power", "%.1f", ((DcMotorEx) vertical).getVelocity());
        telemetry.addData("Vertical power consumption", "%.1f", ((DcMotorEx) vertical).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Vertical Position", "%d", vertical.getCurrentPosition());

        telemetry.update();
    }

    // Read the Robot heading in degrees directly from the IMU
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}