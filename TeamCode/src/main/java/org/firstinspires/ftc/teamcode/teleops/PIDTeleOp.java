package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

@TeleOp(name = "Optimized PID TeleOp", group = "Linear OpMode")
public class PIDTeleOp extends LinearOpMode {

    private DcMotor motorFrontLeft = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackRight = null;

    private IMU imu = null;

    private PIDController turnPID;

    private double targetHeading = 0.0; // Desired heading in degrees

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        initPIDController();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        imu.resetYaw();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Get joystick inputs
            double y = -gamepad1.left_stick_y; // Forward/Backward
            double x = gamepad1.left_stick_x * 1.1; // Strafe, adjust for imperfect strafing

            // Update target heading based on right stick input
            double turnInput = -gamepad1.right_stick_x; // Inverted to match heading direction
            double turnRate = turnInput * 30; // Maximum turn rate in degrees per second
            targetHeading += turnRate * getRuntime(); // Update target heading

            // Normalize target heading to be within -180 to +180 degrees
            targetHeading = normalizeAngle(targetHeading);

            // Get current heading from IMU
            double currentHeading = imu
                .getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.DEGREES);

            // Calculate heading error
            double headingError = getHeadingError(
                targetHeading,
                currentHeading
            );

            // Calculate PID output for rotation
            double rotationOutput = turnPID.calculate(headingError, 0);

            // Use the drive calculations from easyteleop.java with PID rotation control
            double denominator = Math.max(
                Math.abs(y) + Math.abs(x) + Math.abs(rotationOutput),
                1
            );
            double frontLeftPower = (y + x + rotationOutput) / denominator;
            double backLeftPower = (y - x + rotationOutput) / denominator;
            double frontRightPower = (y - x - rotationOutput) / denominator;
            double backRightPower = (y + x - rotationOutput) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            // Telemetry for debugging
            telemetry.addData("Current Heading", "%.2f", currentHeading);
            telemetry.addData("Target Heading", "%.2f", targetHeading);
            telemetry.addData("Heading Error", "%.2f", headingError);
            telemetry.addData("FL Power", "%.2f", frontLeftPower);
            telemetry.addData("FR Power", "%.2f", frontRightPower);
            telemetry.addData("BL Power", "%.2f", backLeftPower);
            telemetry.addData("BR Power", "%.2f", backRightPower);
            telemetry.update();

            // Reset runtime for next loop
            resetRuntime();
        }
    }

    private void initHardware() {
        motorFrontLeft = hardwareMap.dcMotor.get(MecanumDrive.leftFrontName);
        motorBackLeft = hardwareMap.dcMotor.get(MecanumDrive.leftRearName);
        motorFrontRight = hardwareMap.dcMotor.get(MecanumDrive.rightFrontName);
        motorBackRight = hardwareMap.dcMotor.get(MecanumDrive.rightRearName);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize IMU
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot =
            new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    private void initPIDController() {
        turnPID = new PIDController(0.02, 0.0, 0.003); // Adjust kP, kI, kD as needed

        turnPID.setOutputLimits(-1, 1);
    }

    private double getHeadingError(double target, double current) {
        double error = target - current;
        // Normalize error to be within -180 to +180 degrees
        error = normalizeAngle(error);
        return error;
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }
}
