package org.firstinspires.ftc.teamcode.navigate;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Main mecanum drive helper for this robot.
 *
 * <p>This class has three jobs:</p>
 * <ul>
 *     <li>manual mecanum driving</li>
 *     <li>snap-turn heading control using the IMU</li>
 *     <li>odometry-backed target navigation such as "go here and face this direction"</li>
 * </ul>
 *
 * <p>The original public driving behavior is preserved. New target-navigation functionality is
 * layered on top as additional methods so existing callers do not need to change.</p>
 */
public class snap_Mecanum_mover {
    // Predefined snap headings for the trigger shortcuts.
    // In this codebase, 0 radians means field north.
    private static final double RIGHT_TRIGGER_TARGET_RADIANS = -Math.PI / 2.0;
    private static final double LEFT_TRIGGER_TARGET_RADIANS = Math.PI / 2.0;

    // Snap-turn controller tolerances and shaping constants.
    private static final double TURN_STOP_DEADBAND_RADIANS = Math.toRadians(1.0);
    private static final double SETTLE_TOLERANCE_RADIANS = Math.toRadians(2.0);
    private static final double BRAKE_ZONE_RADIANS = Math.toRadians(45.0);
    private static final double FINE_CONTROL_ZONE_RADIANS = Math.toRadians(6.0);
    private static final double SNAP_TURN_KP = 0.76;
    //private static final double SNAP_TURN_KD = 0.60;
    private static final double SNAP_TURN_KD = 0.35;
    private static final double SNAP_TURN_MAX_POWER = 0.70;
    private static final double SNAP_TURN_BRAKE_MAX_POWER = 0.70;
    private static final double SNAP_TURN_SETTLE_MAX_POWER = 0.17;
    private static final double SNAP_TURN_MIN_POWER = 0.14;
    private static final double SNAP_TURN_BRAKE_MIN_POWER = 0.10;
    private static final double MAX_SETTLED_TURN_RATE_RADIANS = Math.toRadians(5.0);
    private static final int REQUIRED_SETTLED_LOOPS = 7;
    private static final double TRIGGER_THRESHOLD = 0.5;
    private static final double MANUAL_ROTATE_DEADBAND = 0.05;
    private static final double SNAP_TURN_MAX_ERROR_RATE_RADIANS_PER_SECOND = Math.toRadians(300.0);
    private static final double SNAP_TURN_ERROR_RATE_FILTER_ALPHA = 0.72;
    private static final double SNAP_TURN_MAX_COMMAND_SLEW_PER_SECOND = 3.2;

    // Separate tuning values for odometry-driven point-to-point navigation.
    private static final double TARGET_POSITION_TOLERANCE_INCHES = 1.5;
    private static final double TARGET_HEADING_TOLERANCE_RADIANS = Math.toRadians(3.0);
    private static final double TARGET_TRANSLATION_KP = 0.035;
    private static final double TARGET_HEADING_KP = 0.018;
    private static final double TARGET_HEADING_KD = 0.0015;
    private static final double TARGET_MAX_TRANSLATION_COMMAND = 0.65;
    private static final double TARGET_MIN_TRANSLATION_COMMAND = 0.12;
    private static final double TARGET_MAX_ROTATION_COMMAND = 0.55;
    private static final double TARGET_MIN_ROTATION_COMMAND = 0.10;

    private final DcMotor frontLeftDrive;
    private final DcMotor frontRightDrive;
    private final DcMotor backLeftDrive;
    private final DcMotor backRightDrive;
    private final IMU imu;

    // Runtime state for the trigger-based snap-turn controller.
    private boolean snapTurnActive = false;
    private double targetHeadingRadians = 0.0;
    private boolean leftTriggerWasPressed = false;
    private boolean rightTriggerWasPressed = false;
    private double previousHeadingErrorRadians = 0.0;
    private long previousSnapUpdateTimeNanos = 0L;
    private double lastTurnRateRadiansPerSecond = 0.0;
    private double filteredHeadingErrorRateRadiansPerSecond = 0.0;
    private double previousRotateCommand = 0.0;
    private boolean snapTurnSettling = false;
    private int settledLoopCount = 0;

    // Odometry is created only if a caller actually uses localization/navigation features.
    private MecanumWheelMotorOdometry odometry;

    // Active target-pose state for "go here and face this way" navigation.
    private MecanumWheelMotorOdometry.Pose2d targetPose;
    private boolean targetNavigationActive = false;
    private double targetPositionToleranceInches = TARGET_POSITION_TOLERANCE_INCHES;
    private double targetHeadingToleranceRadians = TARGET_HEADING_TOLERANCE_RADIANS;
    private double targetTranslationKp = TARGET_TRANSLATION_KP;
    private double targetHeadingKp = TARGET_HEADING_KP;
    private double targetHeadingKd = TARGET_HEADING_KD;
    private double targetMaxTranslationCommand = TARGET_MAX_TRANSLATION_COMMAND;
    private double targetMinTranslationCommand = TARGET_MIN_TRANSLATION_COMMAND;
    private double targetMaxRotationCommand = TARGET_MAX_ROTATION_COMMAND;
    private double targetMinRotationCommand = TARGET_MIN_ROTATION_COMMAND;
    private double targetDistanceInches = 0.0;
    private double targetHeadingErrorRadians = 0.0;
    private double previousTargetHeadingErrorRadians = 0.0;
    private long previousTargetNavigationUpdateTimeNanos = 0L;

    // Stores the last command generated by target navigation for debugging/telemetry.
    private NavigationCommand lastTargetNavigationCommand = new NavigationCommand(0.0, 0.0, 0.0);

    /**
     * Constructs the drive helper around the already-mapped drivetrain motors and IMU.
     */
    public snap_Mecanum_mover(
            DcMotor frontLeftDrive,
            DcMotor frontRightDrive,
            DcMotor backLeftDrive,
            DcMotor backRightDrive,
            IMU imu
    ) {
        this.frontLeftDrive = frontLeftDrive;
        this.frontRightDrive = frontRightDrive;
        this.backLeftDrive = backLeftDrive;
        this.backRightDrive = backRightDrive;
        this.imu = imu;
    }

    /**
     * Driver-control loop used by the existing TeleOp code.
     *
     * <p>This keeps the original behavior:</p>
     * <ul>
     *     <li>left stick = translation</li>
     *     <li>right stick X = manual turn</li>
     *     <li>left bumper = robot-relative drive</li>
     *     <li>otherwise = field-relative drive</li>
     *     <li>left/right triggers = snap-turn shortcuts</li>
     * </ul>
     */
    public void loop(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, boolean snap_to_tag) {
        /*
        telemetry.addLine("Press A to reset Yaw");
        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");
        telemetry.addLine("Left trigger snaps to 90° clockwise from field north");
        telemetry.addLine("Right trigger snaps to 90° counterclockwise from field north");
        */

        // If another routine is temporarily taking over motion control (for example tag snapping),
        // skip the normal driver loop so commands do not fight each other.
        if (snap_to_tag) {
            return;
        }

        // Resetting yaw redefines the robot's current facing direction as zero.
        if (gamepad1.a) {
            imu.resetYaw();
            snapTurnActive = false;
        }

        boolean leftTriggerPressed = gamepad1.left_trigger > TRIGGER_THRESHOLD;
        boolean rightTriggerPressed = gamepad1.right_trigger > TRIGGER_THRESHOLD;

        // Start a snap turn only on the trigger press edge, not every loop while held.
        if (leftTriggerPressed && !rightTriggerWasPressed) {
            startSnapTurn(LEFT_TRIGGER_TARGET_RADIANS);
        } else if (rightTriggerPressed && !leftTriggerWasPressed) {
            startSnapTurn(RIGHT_TRIGGER_TARGET_RADIANS);
        }

        leftTriggerWasPressed = leftTriggerPressed;
        rightTriggerWasPressed = rightTriggerPressed;

        // Manual turn input overrides the snap-turn controller immediately.
        double rotateCommand = gamepad1.right_stick_x;
        if (Math.abs(rotateCommand) > MANUAL_ROTATE_DEADBAND) {
            snapTurnActive = false;
        } else if (snapTurnActive) {
            // Negated here to match the sign convention already used by the rest of the drive code.
            rotateCommand = -getSnapTurnCommand(telemetry);
        }

        //telemetry.addData("Heading (deg)", Math.toDegrees(getCurrentHeadingRadians()));
        //telemetry.addData("Snap turn", snapTurnActive ? "ACTIVE" : "OFF");
        /*
        if (snapTurnActive) {
            telemetry.addData("Target heading (deg)", Math.toDegrees(targetHeadingRadians));
            telemetry.addData("Heading error (deg)", Math.toDegrees(AngleUnit.normalizeRadians(targetHeadingRadians - getCurrentHeadingRadians())));
            telemetry.addData("Turn rate (deg/s)", Math.toDegrees(lastTurnRateRadiansPerSecond));
            telemetry.addData("Settling", snapTurnSettling ? "YES" : "NO");
        }
        */
        if (gamepad1.left_bumper) {
            // Robot-relative mode: stick directions are interpreted from the robot's point of view.
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, rotateCommand);
        } else {
            // Field-relative mode: stick directions are interpreted from the field's point of view.
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, rotateCommand);
        }
    }

    /**
     * Arms the snap-turn controller to rotate toward a desired heading.
     */
    private void startSnapTurn(double headingRadians) {
        targetHeadingRadians = AngleUnit.normalizeRadians(headingRadians);
        previousHeadingErrorRadians = AngleUnit.normalizeRadians(targetHeadingRadians - getCurrentHeadingRadians());
        previousSnapUpdateTimeNanos = System.nanoTime();
        lastTurnRateRadiansPerSecond = 0.0;
        filteredHeadingErrorRateRadiansPerSecond = 0.0;
        previousRotateCommand = 0.0;
        snapTurnSettling = false;
        settledLoopCount = 0;
        snapTurnActive = true;
    }

    /**
     * Computes the current snap-turn rotation command.
     *
     * <p>This is a damped PD-style heading controller with a brake zone and a low-power settle
     * phase near the target to reduce overshoot.</p>
     */
    private double getSnapTurnCommand(Telemetry telemetry) {
        double headingError = AngleUnit.normalizeRadians(targetHeadingRadians - getCurrentHeadingRadians());
        /*
        telemetry.addData("heading error value", Math.abs(headingError));
        telemetry.addData("deadband", TURN_STOP_DEADBAND_RADIANS);
        telemetry.addData("settled loop count", settledLoopCount);
        */
        long nowNanos = System.nanoTime();
        double deltaTimeSeconds = (nowNanos - previousSnapUpdateTimeNanos) / 1_000_000_000.0;

        if (deltaTimeSeconds <= 0.0) {
            deltaTimeSeconds = 0.001;
        }

        double errorDeltaRadians = AngleUnit.normalizeRadians(headingError - previousHeadingErrorRadians);
        double rawErrorRateRadiansPerSecond = errorDeltaRadians / deltaTimeSeconds;
        // Limit impossible derivative spikes from IMU sample jitter or angle wrap transitions.
        rawErrorRateRadiansPerSecond = Range.clip(
                rawErrorRateRadiansPerSecond,
                -SNAP_TURN_MAX_ERROR_RATE_RADIANS_PER_SECOND,
                SNAP_TURN_MAX_ERROR_RATE_RADIANS_PER_SECOND
        );

        // Low-pass filter derivative so tiny heading jitter does not create direction-flip jerk.
        filteredHeadingErrorRateRadiansPerSecond =
                (SNAP_TURN_ERROR_RATE_FILTER_ALPHA * filteredHeadingErrorRateRadiansPerSecond)
                        + ((1.0 - SNAP_TURN_ERROR_RATE_FILTER_ALPHA) * rawErrorRateRadiansPerSecond);
        lastTurnRateRadiansPerSecond = -filteredHeadingErrorRateRadiansPerSecond;

        // Once the heading is close enough, stop the snap-turn controller entirely.
        if (Math.abs(headingError) <= TURN_STOP_DEADBAND_RADIANS) {
            snapTurnActive = false;
            snapTurnSettling = false;
            settledLoopCount = 0;
            previousHeadingErrorRadians = headingError;
            previousSnapUpdateTimeNanos = nowNanos;
            lastTurnRateRadiansPerSecond = 0.0;
            filteredHeadingErrorRateRadiansPerSecond = 0.0;
            previousRotateCommand = 0.0;
            return 0.0;
        }

        boolean overshotTarget = Math.signum(previousHeadingErrorRadians) != Math.signum(headingError)
                && Math.abs(previousHeadingErrorRadians) > TURN_STOP_DEADBAND_RADIANS;

        // If we crossed the target or entered the fine-control zone, switch into settle mode.
        if (overshotTarget || Math.abs(headingError) <= FINE_CONTROL_ZONE_RADIANS) {
            snapTurnSettling = true;
        }

        boolean headingWithinTolerance = Math.abs(headingError) <= SETTLE_TOLERANCE_RADIANS;
        boolean turnRateIsLow = Math.abs(lastTurnRateRadiansPerSecond) <= MAX_SETTLED_TURN_RATE_RADIANS;

        if (headingWithinTolerance && turnRateIsLow) {
            settledLoopCount++;
        } else {
            settledLoopCount = 0;
        }

        // Require several consecutive settled loops before considering the robot stable.
        if (settledLoopCount >= REQUIRED_SETTLED_LOOPS
                && Math.abs(headingError) <= TURN_STOP_DEADBAND_RADIANS) {
            snapTurnActive = false;
            snapTurnSettling = false;
            settledLoopCount = 0;
            previousHeadingErrorRadians = headingError;
            previousSnapUpdateTimeNanos = nowNanos;
            lastTurnRateRadiansPerSecond = 0.0;
            filteredHeadingErrorRateRadiansPerSecond = 0.0;
            previousRotateCommand = 0.0;
            return 0.0;
        }

        double rotateCommand =
                (headingError * SNAP_TURN_KP)
                        + (filteredHeadingErrorRateRadiansPerSecond * SNAP_TURN_KD);

        double absoluteError = Math.abs(headingError);
        double maxAllowedPower;
        double minAllowedPower = 0.0;

        // Large errors can use more power. Smaller errors taper down to avoid oscillation.
        if (snapTurnSettling) {
            maxAllowedPower = SNAP_TURN_SETTLE_MAX_POWER;
        } else if (absoluteError <= BRAKE_ZONE_RADIANS) {
            double brakeBlend = Range.clip(absoluteError / BRAKE_ZONE_RADIANS, 0.0, 1.0);
            maxAllowedPower = SNAP_TURN_BRAKE_MAX_POWER
                    + (SNAP_TURN_MAX_POWER - SNAP_TURN_BRAKE_MAX_POWER) * brakeBlend;
            minAllowedPower = SNAP_TURN_BRAKE_MIN_POWER * brakeBlend;
        } else {
            maxAllowedPower = SNAP_TURN_MAX_POWER;
            minAllowedPower = SNAP_TURN_MIN_POWER;
        }

        rotateCommand = Range.clip(rotateCommand, -maxAllowedPower, maxAllowedPower);

        if (minAllowedPower > 0.0 && Math.abs(rotateCommand) < minAllowedPower) {
            rotateCommand = Math.copySign(minAllowedPower, rotateCommand);
        }

        // Slew-limit the rotation command so output does not jump sharply between loops.
        double maxDeltaCommand = SNAP_TURN_MAX_COMMAND_SLEW_PER_SECOND * deltaTimeSeconds;
        rotateCommand = Range.clip(
                rotateCommand,
                previousRotateCommand - maxDeltaCommand,
                previousRotateCommand + maxDeltaCommand
        );
        previousRotateCommand = rotateCommand;

        // Save state for the next derivative calculation.
        previousHeadingErrorRadians = headingError;
        previousSnapUpdateTimeNanos = nowNanos;

        return rotateCommand;
    }

    /**
     * Reads the robot yaw from the IMU in radians.
     */
    private double getCurrentHeadingRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /**
     * Field-relative mecanum drive.
     *
     * <p>forward/right are interpreted in field coordinates. This method rotates the requested
     * vector by the robot heading and then delegates to the normal robot-relative drive method.</p>
     */
    public void driveFieldRelative(double forward, double right, double rotate) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        theta = AngleUnit.normalizeRadians(theta - getCurrentHeadingRadians());

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive(newForward, newRight, rotate);
    }

    /**
     * Robot-relative mecanum drive.
     *
     * <p>Parameters:</p>
     * <ul>
     *     <li>forward: +forward / -backward</li>
     *     <li>right: +strafe right / -strafe left</li>
     *     <li>rotate: +CCW / -CW based on the existing drive sign convention</li>
     * </ul>
     */
    public void drive(double forward, double right, double rotate) {
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        // Normalize so no individual wheel command exceeds magnitude 1.0.
        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }

    /**
     * Lazily creates the odometry helper so this class remains backward compatible with existing
     * callers that only use manual and snap-turn driving.
     */
    public MecanumWheelMotorOdometry getOdometry() {
        if (odometry == null) {
            odometry = new MecanumWheelMotorOdometry(
                    frontLeftDrive,
                    frontRightDrive,
                    backLeftDrive,
                    backRightDrive,
                    imu
            );
        }

        return odometry;
    }

    public void initializeOdometry() {
        getOdometry().initialize();
    }

    /**
     * Updates the odometry estimate from wheel encoders and IMU.
     * Call once per loop if you want fresh localization data without using target navigation.
     */
    public void updateOdometry() {
        getOdometry().update();
    }

    /**
     * Manually defines the robot's field pose in inches/radians.
     *
     * <p>Use this at the start of autonomous if you know the real starting location.</p>
     */
    public void resetOdometryPose(double xInches, double yInches, double headingRadians) {
        getOdometry().resetPose(xInches, yInches, headingRadians);
    }

    /**
     * Same as resetOdometryPose(), but accepts feet and degrees for convenience.
     */
    public void resetOdometryPoseFeet(double xFeet, double yFeet, double headingDegrees) {
        getOdometry().resetPoseFeet(xFeet, yFeet, headingDegrees);
    }

    /**
     * Sets the chassis geometry used by wheel-based odometry.
     * Measure and tune these values on the real robot for best localization accuracy.
     */
    public void setOdometryGeometry(double trackWidthInches, double wheelBaseInches) {
        getOdometry().setRobotGeometry(trackWidthInches, wheelBaseInches);
    }

    /**
     * Sets the assumed field size in feet.
     */
    public void setOdometryFieldSizeFeet(double fieldSizeFeet) {
        getOdometry().setFieldSizeFeet(fieldSizeFeet);
    }

    /**
     * Sets the assumed field size in inches.
     */
    public void setOdometryFieldSizeInches(double fieldSizeInches) {
        getOdometry().setFieldSizeInches(fieldSizeInches);
    }

    /**
     * Controls whether odometry heading comes from the IMU or from wheel-only estimation.
     * IMU is generally the better choice for FTC mecanum robots.
     */
    public void setOdometryUseImuHeading(boolean useImuHeading) {
        getOdometry().setUseImuHeading(useImuHeading);
    }

    /**
     * Dumps all odometry/localization telemetry to the provided telemetry stream.
     */
    public void addOdometryTelemetry(Telemetry telemetry) {
        getOdometry().addTelemetry(telemetry);
    }

    /**
     * Tunes the proportional/derivative gains used for point-to-point navigation.
     */
    public void setTargetNavigationGains(double translationKp, double headingKp, double headingKd) {
        this.targetTranslationKp = Math.max(0.0, translationKp);
        this.targetHeadingKp = Math.max(0.0, headingKp);
        this.targetHeadingKd = Math.max(0.0, headingKd);
    }

    /**
     * Sets command limits for autonomous target navigation.
     *
     * <p>These values cap and floor the generated movement commands so the robot neither stalls
     * too early nor commands excessive power.</p>
     */
    public void setTargetNavigationLimits(
            double maxTranslationCommand,
            double minTranslationCommand,
            double maxRotationCommand,
            double minRotationCommand
    ) {
        this.targetMaxTranslationCommand = Range.clip(maxTranslationCommand, 0.0, 1.0);
        this.targetMinTranslationCommand = Range.clip(minTranslationCommand, 0.0, this.targetMaxTranslationCommand);
        this.targetMaxRotationCommand = Range.clip(maxRotationCommand, 0.0, 1.0);
        this.targetMinRotationCommand = Range.clip(minRotationCommand, 0.0, this.targetMaxRotationCommand);
    }

    /**
     * Sets the position/heading tolerances used to decide when the robot has reached its target.
     */
    public void setTargetNavigationTolerances(double positionToleranceInches, double headingToleranceDegrees) {
        this.targetPositionToleranceInches = Math.max(0.0, positionToleranceInches);
        this.targetHeadingToleranceRadians = Math.toRadians(Math.max(0.0, headingToleranceDegrees));
    }

    /**
     * Sets the destination position and the direction the robot should be facing when it gets there.
     */
    public void setTargetPose(double xInches, double yInches, double headingRadians) {
        MecanumWheelMotorOdometry activeOdometry = getOdometry();

        // If odometry is configured to clamp to the known field, keep the target inside it.
        double targetXInches = activeOdometry.isClampPoseToField()
                ? Range.clip(xInches, 0.0, activeOdometry.getFieldSizeInches())
                : xInches;
        double targetYInches = activeOdometry.isClampPoseToField()
                ? Range.clip(yInches, 0.0, activeOdometry.getFieldSizeInches())
                : yInches;

        targetPose = new MecanumWheelMotorOdometry.Pose2d(targetXInches, targetYInches, headingRadians);
        targetNavigationActive = true;

        // Reset derivative state whenever a brand-new target is chosen.
        previousTargetHeadingErrorRadians = 0.0;
        previousTargetNavigationUpdateTimeNanos = System.nanoTime();
    }

    /**
     * Convenience overload using feet/degrees instead of inches/radians.
     */
    public void setTargetPoseFeet(double xFeet, double yFeet, double headingDegrees) {
        setTargetPose(xFeet * 12.0, yFeet * 12.0, Math.toRadians(headingDegrees));
    }

    /**
     * Changes only the final facing direction while preserving the current target X/Y location.
     * If no target position exists yet, the robot's current location becomes the target position.
     */
    public void setTargetDirectionRadians(double headingRadians) {
        MecanumWheelMotorOdometry activeOdometry = getOdometry();
        double targetXInches = targetPose != null ? targetPose.xInches : activeOdometry.getXInches();
        double targetYInches = targetPose != null ? targetPose.yInches : activeOdometry.getYInches();
        setTargetPose(targetXInches, targetYInches, headingRadians);
    }

    public void setTargetDirectionDegrees(double headingDegrees) {
        setTargetDirectionRadians(Math.toRadians(headingDegrees));
    }

    /**
     * Convenience helper for the common case of ending the move facing field north.
     * 0 degrees in this codebase corresponds to field north.
     */
    public void setTargetDirectionFieldNorth() {
        setTargetDirectionDegrees(0.0);
    }

    /**
     * Target a field position in inches and finish facing north.
     */
    public void setTargetPoseFacingFieldNorth(double xInches, double yInches) {
        setTargetPose(xInches, yInches, 0.0);
    }

    /**
     * Target a field position in feet and finish facing north.
     */
    public void setTargetPoseFeetFacingFieldNorth(double xFeet, double yFeet) {
        setTargetPoseFeet(xFeet, yFeet, 0.0);
    }

    /**
     * Returns the currently stored target pose, or null if no target is active.
     */
    public MecanumWheelMotorOdometry.Pose2d getTargetPose() {
        return targetPose == null ? null : new MecanumWheelMotorOdometry.Pose2d(
                targetPose.xInches,
                targetPose.yInches,
                targetPose.headingRadians
        );
    }

    /**
     * Returns the last navigation command generated by navigateToTarget().
     * Useful for debugging tuning issues.
     */
    public NavigationCommand getLastTargetNavigationCommand() {
        return new NavigationCommand(
                lastTargetNavigationCommand.forward,
                lastTargetNavigationCommand.right,
                lastTargetNavigationCommand.rotate
        );
    }

    /**
     * Combined helper that stores a target pose and immediately runs one navigation update toward it.
     *
     * <p>Returns true when the robot is already at the target after the update.</p>
     */
    public boolean navigateTo(double xInches, double yInches, double headingRadians) {
        boolean targetChanged = targetPose == null
                || Math.abs(targetPose.xInches - xInches) > 0.001
                || Math.abs(targetPose.yInches - yInches) > 0.001
                || Math.abs(AngleUnit.normalizeRadians(targetPose.headingRadians - headingRadians)) > Math.toRadians(0.1);

        if (targetChanged) {
            setTargetPose(xInches, yInches, headingRadians);
        } else {
            targetNavigationActive = true;
        }

        return navigateToTarget();
    }

    /**
     * Drives to the current target pose using wheel-motor odometry for position feedback and this
     * class's existing mecanum drive methods for motion output.
     */
    public boolean navigateToTarget() {
        MecanumWheelMotorOdometry activeOdometry = getOdometry();
        activeOdometry.update();

        // No target means there is nothing to do; stop cleanly and report success.
        if (targetPose == null) {
            stopTargetNavigation();
            targetNavigationActive = false;
            return true;
        }

        updateTargetErrors(activeOdometry);

        // If we are already close enough in both position and heading, stop driving.
        if (isAtTarget()) {
            stopTargetNavigation();
            targetNavigationActive = false;
            return true;
        }

        // Position controller: move proportionally toward the target point in field coordinates.
        double rightCommand = (targetPose.xInches - activeOdometry.getXInches()) * targetTranslationKp;
        double forwardCommand = (targetPose.yInches - activeOdometry.getYInches()) * targetTranslationKp;
        double translationMagnitude = Math.hypot(forwardCommand, rightCommand);

        // Once close enough to the point, stop translating and just clean up heading.
        if (targetDistanceInches <= targetPositionToleranceInches) {
            forwardCommand = 0.0;
            rightCommand = 0.0;
        } else {
            // Clamp translation so the robot remains controllable and consistent.
            if (translationMagnitude > targetMaxTranslationCommand && translationMagnitude > 0.0) {
                double scale = targetMaxTranslationCommand / translationMagnitude;
                forwardCommand *= scale;
                rightCommand *= scale;
                translationMagnitude = targetMaxTranslationCommand;
            }

            if (translationMagnitude < targetMinTranslationCommand && translationMagnitude > 0.0) {
                double scale = targetMinTranslationCommand / translationMagnitude;
                forwardCommand *= scale;
                rightCommand *= scale;
            }

            // Reduce translation while the heading error is large to keep the path more stable.
            double headingScale = Range.clip(1.0 - (Math.abs(targetHeadingErrorRadians) / Math.PI), 0.25, 1.0);
            forwardCommand *= headingScale;
            rightCommand *= headingScale;
        }

        long nowNanos = System.nanoTime();
        double deltaTimeSeconds = (nowNanos - previousTargetNavigationUpdateTimeNanos) / 1_000_000_000.0;
        if (deltaTimeSeconds <= 0.0) {
            deltaTimeSeconds = 0.001;
        }

        // Heading controller: rotate toward the desired final heading while driving the path.
        double headingErrorRate = AngleUnit.normalizeRadians(targetHeadingErrorRadians - previousTargetHeadingErrorRadians) / deltaTimeSeconds;
        double rotateCommand = (targetHeadingErrorRadians * targetHeadingKp) + (headingErrorRate * targetHeadingKd);
        rotateCommand = Range.clip(rotateCommand, -targetMaxRotationCommand, targetMaxRotationCommand);

        if (Math.abs(targetHeadingErrorRadians) <= targetHeadingToleranceRadians) {
            rotateCommand = 0.0;
        } else if (Math.abs(rotateCommand) < targetMinRotationCommand) {
            rotateCommand = Math.copySign(targetMinRotationCommand, rotateCommand);
        }

        // Slow down near the point so final approach and heading settle are smoother.
        if (targetDistanceInches <= Math.max(targetPositionToleranceInches * 2.0, 3.0)) {
            forwardCommand *= 0.4;
            rightCommand *= 0.4;
        }

        // Reuse the existing field-relative drive implementation instead of commanding wheels here.
        driveFieldRelative(forwardCommand, rightCommand, rotateCommand);
        lastTargetNavigationCommand = new NavigationCommand(forwardCommand, rightCommand, rotateCommand);
        previousTargetHeadingErrorRadians = targetHeadingErrorRadians;
        previousTargetNavigationUpdateTimeNanos = nowNanos;
        targetNavigationActive = true;

        // Target navigation and trigger-based snap-turn are both heading controllers, so disable the
        // trigger-based one while autonomous target motion is active.
        snapTurnActive = false;
        return false;
    }

    /**
     * Returns true when the active target is within both the configured position tolerance and the
     * configured heading tolerance.
     */
    public boolean isAtTarget() {
        if (targetPose == null) {
            return true;
        }

        updateTargetErrors(getOdometry());
        return targetDistanceInches <= targetPositionToleranceInches
                && Math.abs(targetHeadingErrorRadians) <= targetHeadingToleranceRadians;
    }

    /**
     * Clears the active target and stops any navigation-generated motion.
     */
    public void cancelTarget() {
        targetPose = null;
        targetNavigationActive = false;
        targetDistanceInches = 0.0;
        targetHeadingErrorRadians = 0.0;
        previousTargetHeadingErrorRadians = 0.0;
        stopTargetNavigation();
    }

    /**
     * Adds target-navigation telemetry for easy tuning on Driver Station.
     */
    public void addTargetNavigationTelemetry(Telemetry telemetry) {
        telemetry.addLine("=== SNAP TARGET NAVIGATION ===");
        telemetry.addData("Target navigation active", targetNavigationActive ? "YES" : "NO");
        telemetry.addData("At target", isAtTarget() ? "YES" : "NO");

        if (targetPose != null) {
            telemetry.addData("Target X / Y (in)", "%.2f / %.2f", targetPose.xInches, targetPose.yInches);
            telemetry.addData("Target X / Y (ft)", "%.2f / %.2f", targetPose.getXFeet(), targetPose.getYFeet());
            telemetry.addData("Target heading (deg)", "%.2f", targetPose.getHeadingDegrees());
            telemetry.addData("Target distance (in)", "%.2f", targetDistanceInches);
            telemetry.addData("Target heading error (deg)", "%.2f", Math.toDegrees(targetHeadingErrorRadians));
        }

        telemetry.addData("Last nav F / R / Turn", "%.3f / %.3f / %.3f",
                lastTargetNavigationCommand.forward,
                lastTargetNavigationCommand.right,
                lastTargetNavigationCommand.rotate);
    }

    /**
     * Recomputes distance and heading error from the current odometry pose to the stored target.
     */
    private void updateTargetErrors(MecanumWheelMotorOdometry activeOdometry) {
        if (targetPose == null) {
            targetDistanceInches = 0.0;
            targetHeadingErrorRadians = 0.0;
            return;
        }

        double xErrorInches = targetPose.xInches - activeOdometry.getXInches();
        double yErrorInches = targetPose.yInches - activeOdometry.getYInches();
        targetDistanceInches = Math.hypot(xErrorInches, yErrorInches);
        targetHeadingErrorRadians = AngleUnit.normalizeRadians(targetPose.headingRadians - activeOdometry.getHeadingRadians());
    }

    /**
     * Stops motion commanded by the target-navigation layer.
     */
    private void stopTargetNavigation() {
        driveFieldRelative(0.0, 0.0, 0.0);
        lastTargetNavigationCommand = new NavigationCommand(0.0, 0.0, 0.0);
    }

    /**
     * Simple immutable container for the last generated navigation command.
     */
    public static class NavigationCommand {
        public final double forward;
        public final double right;
        public final double rotate;

        public NavigationCommand(double forward, double right, double rotate) {
            this.forward = forward;
            this.right = right;
            this.rotate = rotate;
        }
    }
}

