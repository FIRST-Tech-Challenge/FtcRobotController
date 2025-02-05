package org.firstinspires.ftc.teamcode.opmodes.test.DriveTests.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Arm+DriveTest", group = "Linear OpMode")
@Disabled
public class TeleOpTest1 extends LinearOpMode  {

    // Declare OpMode members for the 4 motors, IMU, and elapsed time
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private BNO055IMU imu;
    private DcMotor linearSlideMotor = null;
    private DcMotor ArmMotor = null;

    // Motor power settings
    private static final double SLIDE_POWER = 0.8;   // Adjust based on required speed

    //@Override
    public void arminit() {
        // Initialize the linear slide motor from the hardware map
        linearSlideMotor = hardwareMap.get(DcMotor.class, "linearSlideMotor");
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");

        // Set motor direction if necessary (adjust based on your setup)
        linearSlideMotor.setDirection(DcMotor.Direction.REVERSE);
        ArmMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior to brake so it holds position when stopped
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up the ArmMotor's encoder position to 0
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset encoder, setting position to 0
        ArmMotor.setTargetPosition(0);                             // Set initial target to position 0
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);         // Enable position control mode
    }

    @Override
    public void runOpMode() {

        // Initialize the hardware variables for the 4 mecanum motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeftMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRightMotor");


        // Set motor directions: Reverse the motors on one side to ensure correct movement
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Initialize IMU hardware and parameters
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(imuParameters);

        // Display initialization status on Driver Station
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Get input from the joysticks
            double y = -gamepad1.left_stick_y;   // forward/backward
            double x = gamepad1.left_stick_x;    // strafe (left/right)
            double rx = gamepad1.right_stick_x;  // rotation (turn)

            // Reset IMU yaw if the options button is pressed
            if (gamepad1.options) {
                imu.initialize(imuParameters);
            }

            // Retrieve the robot heading in radians from the IMU
            double botHeading = imu.getAngularOrientation().firstAngle;

            // Adjust for robot's orientation (field-centric control)
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Optional strafe power adjustment for better control
            rotX *= 1.1;

            // Determine the largest motor power needed for normalization
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            // Set power to each motor
            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);

            // Display telemetry information
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "Front L/R: (%.2f, %.2f), Back L/R: (%.2f, %.2f)",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("Heading", botHeading);
            telemetry.update();


            if (gamepad1.a) {
                ArmMotor.setTargetPosition(0);     // Set target to position 0
                ArmMotor.setPower(0.5);            // Adjust power for controlled movement
            }

            // Manual control for linearSlideMotor
            if (gamepad1.dpad_up) {
                linearSlideMotor.setPower(SLIDE_POWER);
            } else if (gamepad1.dpad_down) {
                linearSlideMotor.setPower(-SLIDE_POWER);
            } else {
                linearSlideMotor.setPower(0);
            }

            // Manual control for ArmMotor
            if (gamepad1.dpad_right) {
                ArmMotor.setPower(SLIDE_POWER);
            } else if (gamepad1.dpad_left) {
                ArmMotor.setPower(-SLIDE_POWER);
            } else if (!gamepad1.a) {  // Only stop ArmMotor if 'A' is not pressed
                ArmMotor.setPower(0);
            }

            // Telemetry for monitoring
            telemetry.addData("Slide Power", linearSlideMotor.getPower());
            telemetry.addData("Arm Power", ArmMotor.getPower());
            telemetry.addData("Arm Position", ArmMotor.getCurrentPosition()); // Display current encoder position
            telemetry.update();
        }
    }

    //@Override
    public void armstop() {
        // Ensure motor is stopped when the OpMode ends
        linearSlideMotor.setPower(0);
        ArmMotor.setPower(0);
    }
}
