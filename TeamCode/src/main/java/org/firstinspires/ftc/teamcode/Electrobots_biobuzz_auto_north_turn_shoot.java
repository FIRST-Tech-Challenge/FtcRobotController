package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.navigate.MecanumWheelMotorOdometry;
import org.firstinspires.ftc.teamcode.navigate.SnapMecanumMover;
import org.firstinspires.ftc.teamcode.scoring.IntakeController;
import org.firstinspires.ftc.teamcode.scoring.ShooterController;

/**
 * Autonomous routine:
 * 1) Move north by 4 wheel rotations.
 * 2) Turn left 90 degrees (face west).
 * 3) Move west by 2 wheel rotations.
 * 4) Spin shooter to target velocity.
 * 5) Once ready, activate windmill and feed for 5 seconds.
 * 6) Stop everything.
 *
 * This class reuses existing robot controllers so tuning changes in shared scoring code
 * are automatically used here too.
 */
@Autonomous(name = "Electrobots Auto NorthTurnShoot", group = "Robot")
public class Electrobots_biobuzz_auto_north_turn_shoot extends LinearOpMode {

    // Cardinal headings in this codebase coordinate system:
    // 0 rad = North (+Y), -90 deg = East (+X), +90 deg = West (-X), 180 deg = South (-Y).
    private static final double NORTH_RADIANS = 0.0;
    private static final double EAST_RADIANS = -Math.PI / 2.0;
    private static final double WEST_RADIANS = Math.PI / 2.0;
    private static final double SOUTH_RADIANS = Math.PI;

    private static final double DEFAULT_MOVE_TIMEOUT_SECONDS = 6.0;
    private static final double DEFAULT_TURN_TIMEOUT_SECONDS = 4.0;
    private static final double SHOOT_AND_FEED_SECONDS = 5.0;

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private IMU imu;

    private SnapMecanumMover snapMecanumMover;
    private ShooterController shooterController;
    private IntakeController intakeController;

    @Override
    public void runOpMode() {
        initDriveHardware();

        snapMecanumMover = new SnapMecanumMover(
                frontLeftDrive,
                frontRightDrive,
                backLeftDrive,
                backRightDrive,
                imu
        );

        shooterController = new ShooterController();
        shooterController.init(hardwareMap);

        intakeController = new IntakeController();
        intakeController.init(hardwareMap);

        // Prepare odometry before autonomous starts.
        snapMecanumMover.initializeOdometry();

        telemetry.addLine("Auto ready: North 4 turns, turn left 90, West 2 turns, shoot + windmill");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopAllActuators();
            return;
        }

        // Step 1: Move north 4 wheel rotations.
        GoNorth(4.0);

        // Step 2: Turn robot left by 90 degrees (from north -> west).
        TurnWest(0.0);

        // Step 3: Move west 2 wheel rotations.
        GoWest(2.0);

        // Step 4/5: Shooter spin-up, then windmill feed for 5 seconds.
        shootAndFeedForWindow(SHOOT_AND_FEED_SECONDS);

        // Step 6: Stop all mechanisms and driving.
        stopAllActuators();
    }

    /** Moves north (+Y field direction) by the given wheel rotations. */
    public void GoNorth(double wheelRotations) {
        moveCardinalByWheelRotations(0.0, rotationsToInches(wheelRotations), DEFAULT_MOVE_TIMEOUT_SECONDS);
    }

    /** Moves south (-Y field direction) by the given wheel rotations. */
    public void GoSouth(double wheelRotations) {
        moveCardinalByWheelRotations(0.0, -rotationsToInches(wheelRotations), DEFAULT_MOVE_TIMEOUT_SECONDS);
    }

    /** Moves east (+X field direction) by the given wheel rotations. */
    public void GoEast(double wheelRotations) {
        moveCardinalByWheelRotations(rotationsToInches(wheelRotations), 0.0, DEFAULT_MOVE_TIMEOUT_SECONDS);
    }

    /** Moves west (-X field direction) by the given wheel rotations. */
    public void GoWest(double wheelRotations) {
        moveCardinalByWheelRotations(-rotationsToInches(wheelRotations), 0.0, DEFAULT_MOVE_TIMEOUT_SECONDS);
    }

    /** Turns robot to a heading based on north with an optional degree offset. */
    public void TurnNorth(double degreesOffset) {
        turnToHeading(NORTH_RADIANS + Math.toRadians(degreesOffset), DEFAULT_TURN_TIMEOUT_SECONDS);
    }

    /** Turns robot to a heading based on east with an optional degree offset. */
    public void TurnEast(double degreesOffset) {
        turnToHeading(EAST_RADIANS + Math.toRadians(degreesOffset), DEFAULT_TURN_TIMEOUT_SECONDS);
    }

    /** Turns robot to a heading based on west with an optional degree offset. */
    public void TurnWest(double degreesOffset) {
        turnToHeading(WEST_RADIANS + Math.toRadians(degreesOffset), DEFAULT_TURN_TIMEOUT_SECONDS);
    }

    /** Turns robot to a heading based on south with an optional degree offset. */
    public void TurnSouth(double degreesOffset) {
        turnToHeading(SOUTH_RADIANS + Math.toRadians(degreesOffset), DEFAULT_TURN_TIMEOUT_SECONDS);
    }

    private void moveCardinalByWheelRotations(double xDeltaInches, double yDeltaInches, double timeoutSeconds) {
        MecanumWheelMotorOdometry.Pose2d pose = getPoseSnapshot();
        double targetX = pose.xInches + xDeltaInches;
        double targetY = pose.yInches + yDeltaInches;
        navigateToPose(targetX, targetY, pose.headingRadians, timeoutSeconds);
    }

    private void turnToHeading(double targetHeadingRadians, double timeoutSeconds) {
        MecanumWheelMotorOdometry.Pose2d pose = getPoseSnapshot();
        navigateToPose(pose.xInches, pose.yInches, targetHeadingRadians, timeoutSeconds);
    }

    private void navigateToPose(double xInches, double yInches, double headingRadians, double timeoutSeconds) {
        snapMecanumMover.setTargetPose(xInches, yInches, headingRadians);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && !isStopRequested() && timer.seconds() <= timeoutSeconds) {
            boolean reached = snapMecanumMover.navigateToTarget();

            MecanumWheelMotorOdometry.Pose2d pose = snapMecanumMover.getOdometry().getPose();
            telemetry.addData("Nav target", "%.1f, %.1f, %.1f deg", xInches, yInches, Math.toDegrees(headingRadians));
            telemetry.addData("Nav pose", "%.1f, %.1f, %.1f deg", pose.xInches, pose.yInches, Math.toDegrees(pose.headingRadians));
            telemetry.addData("Nav reached", reached ? "YES" : "NO");
            telemetry.addData("Nav timeout (s)", "%.1f / %.1f", timer.seconds(), timeoutSeconds);
            telemetry.update();

            if (reached) {
                break;
            }

            idle();
        }

        snapMecanumMover.cancelTarget();
    }

    private void shootAndFeedForWindow(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && !isStopRequested() && timer.seconds() <= seconds) {
            shooterController.applyShooterSpeedControl(true, 0.0);

            boolean shooterReady = shooterController.isReady();
            intakeController.applyIntakeAndWindmillControl(false, false, shooterReady);

            telemetry.addData("Shooter RPM", "%.1f", shooterController.getLastRpm());
            telemetry.addData("Shooter Target", "%.1f", shooterController.getTargetRpm());
            telemetry.addData("Shooter Ready", shooterReady ? "YES" : "NO");
            telemetry.addData("Windmill Active", shooterReady ? "YES" : "NO");
            telemetry.addData("Shoot timer", "%.1f / %.1f", timer.seconds(), seconds);
            telemetry.update();

            idle();
        }

        intakeController.stop();
        shooterController.stop();
    }

    private MecanumWheelMotorOdometry.Pose2d getPoseSnapshot() {
        MecanumWheelMotorOdometry odometry = snapMecanumMover.getOdometry();
        odometry.update();
        return odometry.getPose();
    }

    private double rotationsToInches(double wheelRotations) {
        return wheelRotations * MecanumWheelMotorOdometry.WHEEL_CIRCUMFERENCE_INCHES;
    }

    private void initDriveHardware() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    private void stopAllActuators() {
        if (snapMecanumMover != null) {
            snapMecanumMover.driveFieldRelative(0.0, 0.0, 0.0);
            snapMecanumMover.cancelTarget();
        }

        if (intakeController != null) {
            intakeController.stop();
        }

        if (shooterController != null) {
            shooterController.stop();
        }
    }
}

