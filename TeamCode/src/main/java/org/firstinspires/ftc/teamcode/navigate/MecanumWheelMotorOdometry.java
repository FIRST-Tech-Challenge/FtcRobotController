package org.firstinspires.ftc.teamcode.navigate;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

/**
 * Wheel-motor-based odometry / localization for a mecanum drivetrain.
 *
 * <p>This class is built around the hardware details you provided:</p>
 * <ul>
 *     <li>goBILDA GripForce Mecanum Wheel Set, 104 mm diameter</li>
 *     <li>goBILDA 5202 Series Yellow Jacket motor, 19.2:1, 312 RPM</li>
 *     <li>Nominal encoder scale used here: 537.7 ticks per output-shaft revolution</li>
 * </ul>
 *
 * <p>Field size is assumed to be 12 ft x 12 ft (144 in x 144 in).</p>
 *
 * <p>The class uses the drive motor encoders to estimate translation. For heading, the default
 * behavior is to use the IMU because it is more stable for field-oriented driving and for any
 * higher-level navigation code that may sit on top of this class. A pure wheel heading estimate is
 * still tracked and published for comparison/debugging.</p>
 *
 * <p>Coordinate system used by this class:</p>
 * <ul>
 *     <li>X increases to the robot's field-right / east</li>
 *     <li>Y increases forward / north on the field</li>
 *     <li>0 heading means the robot faces +Y on the field</li>
 *     <li>Positive heading is counterclockwise</li>
 * </ul>
 *
 * <p>Important note: because the codebase does not currently define your robot's exact chassis
 * track width and wheel base, this class includes reasonable default values and setter methods so
 * you can tune them after measuring the real robot.</p>
 */
public class MecanumWheelMotorOdometry {
    public static final double FIELD_SIZE_FEET = 12.0;
    public static final double FIELD_SIZE_INCHES = FIELD_SIZE_FEET * 12.0;

    public static final double WHEEL_DIAMETER_MM = 104.0;
    public static final double WHEEL_DIAMETER_INCHES = WHEEL_DIAMETER_MM / 25.4;
    public static final double WHEEL_CIRCUMFERENCE_INCHES = Math.PI * WHEEL_DIAMETER_INCHES;

    /**
     * goBILDA 5202 19.2:1 motors are typically treated as 537.7 ticks per output-shaft rev.
     * That matches the constant already used elsewhere in this codebase.
     */
    public static final double DEFAULT_TICKS_PER_WHEEL_REVOLUTION = 537.7;

    /**
     * These defaults are placeholders until you measure your specific robot.
     * They mainly affect wheel-only heading estimation.
     */
    public static final double DEFAULT_TRACK_WIDTH_INCHES = 15.0;
    public static final double DEFAULT_WHEEL_BASE_INCHES = 15.0;

    private final DcMotor frontLeftDrive;
    private final DcMotor frontRightDrive;
    private final DcMotor backLeftDrive;
    private final DcMotor backRightDrive;
    private final IMU imu;

    private double ticksPerWheelRevolution = DEFAULT_TICKS_PER_WHEEL_REVOLUTION;
    private double trackWidthInches = DEFAULT_TRACK_WIDTH_INCHES;
    private double wheelBaseInches = DEFAULT_WHEEL_BASE_INCHES;
    private double fieldSizeInches = FIELD_SIZE_INCHES;
    private boolean clampPoseToField = true;
    private boolean useImuHeading = true;

    private boolean initialized = false;

    private int previousFrontLeftTicks;
    private int previousFrontRightTicks;
    private int previousBackLeftTicks;
    private int previousBackRightTicks;

    private double xInches = FIELD_SIZE_INCHES / 2.0;
    private double yInches = FIELD_SIZE_INCHES / 2.0;
    private double headingRadians = 0.0;
    private double imuHeadingRadians = 0.0;
    private double encoderHeadingRadians = 0.0;
    private double previousImuHeadingRadians = 0.0;
    private double previousUsedHeadingRadians = 0.0;

    private double frontLeftDeltaInches;
    private double frontRightDeltaInches;
    private double backLeftDeltaInches;
    private double backRightDeltaInches;
    private double localForwardDeltaInches;
    private double localRightDeltaInches;
    private double fieldXDeltaInches;
    private double fieldYDeltaInches;
    private double encoderHeadingDeltaRadians;
    private double imuHeadingDeltaRadians;

    private double xVelocityInchesPerSecond;
    private double yVelocityInchesPerSecond;
    private double headingVelocityRadiansPerSecond;
    private long previousUpdateTimeNanos = 0L;

    public MecanumWheelMotorOdometry(
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
     * Captures the current encoder and heading values without moving the robot.
     * Call once in init/start after motor directions and IMU are configured.
     */
    public void initialize() {
        previousFrontLeftTicks = frontLeftDrive.getCurrentPosition();
        previousFrontRightTicks = frontRightDrive.getCurrentPosition();
        previousBackLeftTicks = backLeftDrive.getCurrentPosition();
        previousBackRightTicks = backRightDrive.getCurrentPosition();

        imuHeadingRadians = readImuHeadingRadians();
        previousImuHeadingRadians = imuHeadingRadians;
        headingRadians = imuHeadingRadians;
        encoderHeadingRadians = headingRadians;
        previousUsedHeadingRadians = headingRadians;
        previousUpdateTimeNanos = System.nanoTime();
        initialized = true;
    }

    /**
     * Resets the pose estimate to the given field position.
     * This should normally be called before autonomous/path navigation starts.
     */
    public void resetPose(double xInches, double yInches, double headingRadians) {
        if (!initialized) {
            initialize();
        }

        this.xInches = clampPoseToField ? Range.clip(xInches, 0.0, fieldSizeInches) : xInches;
        this.yInches = clampPoseToField ? Range.clip(yInches, 0.0, fieldSizeInches) : yInches;
        this.headingRadians = AngleUnit.normalizeRadians(headingRadians);
        this.imuHeadingRadians = this.headingRadians;
        this.previousImuHeadingRadians = this.headingRadians;
        this.encoderHeadingRadians = this.headingRadians;
        this.previousUsedHeadingRadians = this.headingRadians;

        previousFrontLeftTicks = frontLeftDrive.getCurrentPosition();
        previousFrontRightTicks = frontRightDrive.getCurrentPosition();
        previousBackLeftTicks = backLeftDrive.getCurrentPosition();
        previousBackRightTicks = backRightDrive.getCurrentPosition();
        previousUpdateTimeNanos = System.nanoTime();

        clearMotionDeltas();
    }

    /** Convenience helper for feet-based setup on a 12 ft x 12 ft field. */
    public void resetPoseFeet(double xFeet, double yFeet, double headingDegrees) {
        resetPose(xFeet * 12.0, yFeet * 12.0, Math.toRadians(headingDegrees));
    }

    /**
     * Update odometry from the wheel encoders and IMU.
     * Call once every loop before reading pose or asking the navigator to drive.
     */
    public void update() {
        if (!initialized) {
            initialize();
            return;
        }

        int currentFrontLeftTicks = frontLeftDrive.getCurrentPosition();
        int currentFrontRightTicks = frontRightDrive.getCurrentPosition();
        int currentBackLeftTicks = backLeftDrive.getCurrentPosition();
        int currentBackRightTicks = backRightDrive.getCurrentPosition();

        frontLeftDeltaInches = ticksToInches(currentFrontLeftTicks - previousFrontLeftTicks);
        frontRightDeltaInches = ticksToInches(currentFrontRightTicks - previousFrontRightTicks);
        backLeftDeltaInches = ticksToInches(currentBackLeftTicks - previousBackLeftTicks);
        backRightDeltaInches = ticksToInches(currentBackRightTicks - previousBackRightTicks);

        previousFrontLeftTicks = currentFrontLeftTicks;
        previousFrontRightTicks = currentFrontRightTicks;
        previousBackLeftTicks = currentBackLeftTicks;
        previousBackRightTicks = currentBackRightTicks;

        localForwardDeltaInches = (frontLeftDeltaInches + frontRightDeltaInches + backRightDeltaInches + backLeftDeltaInches) / 4.0;
        localRightDeltaInches = (frontLeftDeltaInches - frontRightDeltaInches + backRightDeltaInches - backLeftDeltaInches) / 4.0;

        encoderHeadingDeltaRadians = (frontLeftDeltaInches - frontRightDeltaInches - backRightDeltaInches + backLeftDeltaInches)
                / (4.0 * getEffectiveTurnRadiusInches());
        encoderHeadingRadians = AngleUnit.normalizeRadians(encoderHeadingRadians + encoderHeadingDeltaRadians);

        imuHeadingRadians = readImuHeadingRadians();
        imuHeadingDeltaRadians = AngleUnit.normalizeRadians(imuHeadingRadians - previousImuHeadingRadians);
        previousImuHeadingRadians = imuHeadingRadians;

        double currentUsedHeadingRadians = useImuHeading ? imuHeadingRadians : encoderHeadingRadians;
        double headingDeltaRadians = useImuHeading ? imuHeadingDeltaRadians : encoderHeadingDeltaRadians;
        double averageHeadingRadians = AngleUnit.normalizeRadians(previousUsedHeadingRadians + (headingDeltaRadians / 2.0));

        fieldXDeltaInches = (localRightDeltaInches * Math.cos(averageHeadingRadians))
                - (localForwardDeltaInches * Math.sin(averageHeadingRadians));
        fieldYDeltaInches = (localRightDeltaInches * Math.sin(averageHeadingRadians))
                + (localForwardDeltaInches * Math.cos(averageHeadingRadians));

        xInches += fieldXDeltaInches;
        yInches += fieldYDeltaInches;
        if (clampPoseToField) {
            xInches = Range.clip(xInches, 0.0, fieldSizeInches);
            yInches = Range.clip(yInches, 0.0, fieldSizeInches);
        }

        headingRadians = AngleUnit.normalizeRadians(currentUsedHeadingRadians);

        long nowNanos = System.nanoTime();
        double deltaTimeSeconds = (nowNanos - previousUpdateTimeNanos) / 1_000_000_000.0;
        if (deltaTimeSeconds > 0.0) {
            xVelocityInchesPerSecond = fieldXDeltaInches / deltaTimeSeconds;
            yVelocityInchesPerSecond = fieldYDeltaInches / deltaTimeSeconds;
            headingVelocityRadiansPerSecond = AngleUnit.normalizeRadians(headingRadians - previousUsedHeadingRadians) / deltaTimeSeconds;
        } else {
            xVelocityInchesPerSecond = 0.0;
            yVelocityInchesPerSecond = 0.0;
            headingVelocityRadiansPerSecond = 0.0;
        }

        previousUpdateTimeNanos = nowNanos;
        previousUsedHeadingRadians = headingRadians;
    }

    /**
     * Sets the robot geometry used by the wheel-only heading estimate.
     * If you measure your robot, update these values for best odometry quality.
     */
    public void setRobotGeometry(double trackWidthInches, double wheelBaseInches) {
        this.trackWidthInches = Math.max(1.0, trackWidthInches);
        this.wheelBaseInches = Math.max(1.0, wheelBaseInches);
    }

    public void setFieldSizeFeet(double fieldSizeFeet) {
        setFieldSizeInches(fieldSizeFeet * 12.0);
    }

    public void setFieldSizeInches(double fieldSizeInches) {
        this.fieldSizeInches = Math.max(1.0, fieldSizeInches);
        if (clampPoseToField) {
            xInches = Range.clip(xInches, 0.0, this.fieldSizeInches);
            yInches = Range.clip(yInches, 0.0, this.fieldSizeInches);
        }
    }

    public void setUseImuHeading(boolean useImuHeading) {
        this.useImuHeading = useImuHeading;
    }

    public void setClampPoseToField(boolean clampPoseToField) {
        this.clampPoseToField = clampPoseToField;
    }

    public void setTicksPerWheelRevolution(double ticksPerWheelRevolution) {
        this.ticksPerWheelRevolution = Math.max(1.0, ticksPerWheelRevolution);
    }

    /**
     * Adds all useful odometry/localization data to telemetry.
     * Call this if you want the full debug view on the Driver Station.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("=== MECANUM WHEEL ODOMETRY ===");
        telemetry.addData("Field size (in)", "%.1f x %.1f", fieldSizeInches, fieldSizeInches);
        telemetry.addData("Pose X / Y (in)", "%.2f / %.2f", xInches, yInches);
        telemetry.addData("Pose X / Y (ft)", "%.2f / %.2f", xInches / 12.0, yInches / 12.0);
        telemetry.addData("Heading used (deg)", "%.2f", Math.toDegrees(headingRadians));
        telemetry.addData("Heading IMU (deg)", "%.2f", Math.toDegrees(imuHeadingRadians));
        telemetry.addData("Heading encoder (deg)", "%.2f", Math.toDegrees(encoderHeadingRadians));
        telemetry.addData("Heading source", useImuHeading ? "IMU" : "WHEEL ENCODERS");

        telemetry.addLine("--- Wheel deltas / loop ---");
        telemetry.addData("FL / FR (in)", "%.3f / %.3f", frontLeftDeltaInches, frontRightDeltaInches);
        telemetry.addData("BL / BR (in)", "%.3f / %.3f", backLeftDeltaInches, backRightDeltaInches);
        telemetry.addData("Local forward / right (in)", "%.3f / %.3f", localForwardDeltaInches, localRightDeltaInches);
        telemetry.addData("Field dX / dY (in)", "%.3f / %.3f", fieldXDeltaInches, fieldYDeltaInches);
        telemetry.addData("Encoder dHeading (deg)", "%.3f", Math.toDegrees(encoderHeadingDeltaRadians));
        telemetry.addData("IMU dHeading (deg)", "%.3f", Math.toDegrees(imuHeadingDeltaRadians));

        telemetry.addLine("--- Velocities ---");
        telemetry.addData("VX / VY (in/s)", "%.2f / %.2f", xVelocityInchesPerSecond, yVelocityInchesPerSecond);
        telemetry.addData("Turn rate (deg/s)", "%.2f", Math.toDegrees(headingVelocityRadiansPerSecond));

        telemetry.addLine("--- Robot / wheel constants ---");
        telemetry.addData("Wheel diameter (in)", "%.3f", WHEEL_DIAMETER_INCHES);
        telemetry.addData("Wheel circumference (in)", "%.3f", WHEEL_CIRCUMFERENCE_INCHES);
        telemetry.addData("Ticks / wheel rev", "%.1f", ticksPerWheelRevolution);
        telemetry.addData("Inches / tick", "%.5f", getInchesPerTick());
        telemetry.addData("Track / wheel base (in)", "%.2f / %.2f", trackWidthInches, wheelBaseInches);
    }

    public Pose2d getPose() {
        return new Pose2d(xInches, yInches, headingRadians);
    }

    public FieldPoseFeet getPoseFeet() {
        return new FieldPoseFeet(xInches / 12.0, yInches / 12.0, headingRadians);
    }

    public double getXInches() {
        return xInches;
    }

    public double getYInches() {
        return yInches;
    }

    public double getHeadingRadians() {
        return headingRadians;
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(headingRadians);
    }

    public double getImuHeadingDegrees() {
        return Math.toDegrees(imuHeadingRadians);
    }

    public double getEncoderHeadingDegrees() {
        return Math.toDegrees(encoderHeadingRadians);
    }

    public double getFieldSizeInches() {
        return fieldSizeInches;
    }

    public boolean isClampPoseToField() {
        return clampPoseToField;
    }

    public String getPoseSummary() {
        return String.format(
                Locale.US,
                "Pose: x=%.2f in, y=%.2f in, heading=%.2f deg",
                xInches,
                yInches,
                Math.toDegrees(headingRadians)
        );
    }

    private double readImuHeadingRadians() {
        if (imu == null) {
            return encoderHeadingRadians;
        }
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    private double ticksToInches(int ticks) {
        return ticks * getInchesPerTick();
    }

    private double getInchesPerTick() {
        return WHEEL_CIRCUMFERENCE_INCHES / ticksPerWheelRevolution;
    }

    private double getEffectiveTurnRadiusInches() {
        return (trackWidthInches + wheelBaseInches) / 2.0;
    }

    private void clearMotionDeltas() {
        frontLeftDeltaInches = 0.0;
        frontRightDeltaInches = 0.0;
        backLeftDeltaInches = 0.0;
        backRightDeltaInches = 0.0;
        localForwardDeltaInches = 0.0;
        localRightDeltaInches = 0.0;
        fieldXDeltaInches = 0.0;
        fieldYDeltaInches = 0.0;
        encoderHeadingDeltaRadians = 0.0;
        imuHeadingDeltaRadians = 0.0;
        xVelocityInchesPerSecond = 0.0;
        yVelocityInchesPerSecond = 0.0;
        headingVelocityRadiansPerSecond = 0.0;
    }

    public static class Pose2d {
        public final double xInches;
        public final double yInches;
        public final double headingRadians;

        public Pose2d(double xInches, double yInches, double headingRadians) {
            this.xInches = xInches;
            this.yInches = yInches;
            this.headingRadians = AngleUnit.normalizeRadians(headingRadians);
        }

        public double getXFeet() {
            return xInches / 12.0;
        }

        public double getYFeet() {
            return yInches / 12.0;
        }

        public double getHeadingDegrees() {
            return Math.toDegrees(headingRadians);
        }
    }

    public static class FieldPoseFeet {
        public final double xFeet;
        public final double yFeet;
        public final double headingRadians;

        public FieldPoseFeet(double xFeet, double yFeet, double headingRadians) {
            this.xFeet = xFeet;
            this.yFeet = yFeet;
            this.headingRadians = AngleUnit.normalizeRadians(headingRadians);
        }

        public double getHeadingDegrees() {
            return Math.toDegrees(headingRadians);
        }
    }
}
