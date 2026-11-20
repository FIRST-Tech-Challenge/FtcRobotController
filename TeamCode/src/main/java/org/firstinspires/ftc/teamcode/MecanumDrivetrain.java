package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Layer 1 Subsystem: four goBILDA 5000-0002-4008 bare motors driving 20T:80T gears,
 * then coaxial 30T:91T belt pulleys to 104-mm goBILDA 30A mecanum wheels
 * (12.1333:1 total reduction).
 * Encoders measure the motor shafts: 28 quadrature ticks per motor revolution.
 *
 * Plain Java class (not an OpMode) so it can be reused across standalone Test OpModes,
 * TeleOp, and Autonomous.
 */
public class MecanumDrivetrain {

    // Bare shaft specs stay in motor units; the external transmission is applied only at the wheel.
    public static final double BARE_MOTOR_TICKS_PER_REV = BareMotorSpecifications.ENCODER_TICKS_PER_REV;
    public static final double BARE_MOTOR_MAX_RPM = BareMotorSpecifications.CONFIGURED_FREE_SPEED_RPM;
    public static final int MOTOR_GEAR_TEETH = 20;
    public static final int INTERMEDIATE_GEAR_TEETH = 80;
    public static final int INTERMEDIATE_PULLEY_TEETH = 30;
    public static final int WHEEL_PULLEY_TEETH = 91;
    public static final double GEAR_REDUCTION = (double) INTERMEDIATE_GEAR_TEETH / MOTOR_GEAR_TEETH;
    public static final double BELT_REDUCTION = (double) WHEEL_PULLEY_TEETH / INTERMEDIATE_PULLEY_TEETH;
    public static final double MOTOR_REVS_PER_WHEEL_REV = GEAR_REDUCTION * BELT_REDUCTION;
    public static final double WHEEL_TICKS_PER_REV = BARE_MOTOR_TICKS_PER_REV * MOTOR_REVS_PER_WHEEL_REV;
    public static final double WHEEL_MAX_RPM = BARE_MOTOR_MAX_RPM / MOTOR_REVS_PER_WHEEL_REV;
    public static final double WHEEL_DIAMETER_MM = 104.0;
    public static final double WHEEL_CIRCUMFERENCE_MM = WHEEL_DIAMETER_MM * Math.PI;
    // Nominal rolling distance only; loaded roller compression and mecanum slip require measurement.
    public static final double MOTOR_TICKS_PER_MM = WHEEL_TICKS_PER_REV / WHEEL_CIRCUMFERENCE_MM;
    public static final double WHEEL_NO_LOAD_SPEED_MM_PER_SEC = WHEEL_MAX_RPM / 60 * WHEEL_CIRCUMFERENCE_MM;
    public static final double LATERAL_STRAFE_MULTIPLIER = 1.1;
    // Installed drivetrain: all six motions were reversed with left REVERSE/right FORWARD.
    // Correct the hardware boundary once for TeleOp, direct wheel tests and Pedro/autonomous.
    public static final DcMotor.Direction LEFT_MOTOR_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction RIGHT_MOTOR_DIRECTION = DcMotor.Direction.REVERSE;
    // TeleOp-only acceleration limit on wheel power (normalized power per second) so fast stick
    // reversals no longer slip the 20T:80T gears. 10/s = 0 to full in 0.1 s, full reverse in 0.2 s.
    public static final double TELEOP_MAX_WHEEL_POWER_CHANGE_PER_SEC = 10.0;

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;

    private double lastFrontLeftPower = 0.0;
    private double lastFrontRightPower = 0.0;
    private double lastBackLeftPower = 0.0;
    private double lastBackRightPower = 0.0;

    public MecanumDrivetrain(HardwareMap hardwareMap) {
        frontLeft = getDriveMotor(hardwareMap, "leftfront", "front_left_drive", "frontLeft");
        frontRight = getDriveMotor(hardwareMap, "rightfront", "front_right_drive", "frontRight");
        backLeft = getDriveMotor(hardwareMap, "leftback", "back_left_drive", "backLeft");
        backRight = getDriveMotor(hardwareMap, "rightback", "back_right_drive", "backRight");

        configureBareMotor(frontLeft, LEFT_MOTOR_DIRECTION);
        configureBareMotor(backLeft, LEFT_MOTOR_DIRECTION);
        configureBareMotor(frontRight, RIGHT_MOTOR_DIRECTION);
        configureBareMotor(backRight, RIGHT_MOTOR_DIRECTION);
    }

    /**
     * Drives the mecanum chassis in robot-centric coordinates.
     *
     * @param axial   Forward (+1.0) / Backward (-1.0) translation
     * @param lateral Right (+1.0) / Left (-1.0) strafe translation
     * @param yaw     Clockwise (+1.0) / Counter-Clockwise (-1.0) rotation
     */
    public void driveRobotCentric(double axial, double lateral, double yaw) {
        double[] powers = robotCentricWheelPowers(axial, lateral, yaw);
        setWheelPowers(powers[0], powers[1], powers[2], powers[3]);
    }

    /**
     * TeleOp drive: same mixing as {@link #driveRobotCentric}, but each wheel power moves at most
     * {@link #TELEOP_MAX_WHEEL_POWER_CHANGE_PER_SEC} x dt from last loop's command, so fast direction
     * changes ramp briefly instead of reversing torque instantly.
     *
     * @param dtSeconds time since the previous drive command (seconds)
     */
    public void driveRobotCentricRateLimited(double axial, double lateral, double yaw, double dtSeconds) {
        double[] target = robotCentricWheelPowers(axial, lateral, yaw);
        double[] limited = rateLimitWheelPowers(
                new double[] {lastFrontLeftPower, lastFrontRightPower, lastBackLeftPower, lastBackRightPower},
                target, TELEOP_MAX_WHEEL_POWER_CHANGE_PER_SEC * Math.max(0.0, dtSeconds));
        setWheelPowers(limited[0], limited[1], limited[2], limited[3]);
    }

    /**
     * Moves {@code previous} toward {@code target} by at most {@code maxStep} on any wheel. All four
     * changes share one scale factor so the chassis keeps heading toward the commanded motion.
     */
    static double[] rateLimitWheelPowers(double[] previous, double[] target, double maxStep) {
        double largestChange = 0.0;
        for (int i = 0; i < target.length; i++) {
            largestChange = Math.max(largestChange, Math.abs(target[i] - previous[i]));
        }
        double scale = largestChange <= maxStep ? 1.0 : maxStep / largestChange;
        double[] limited = new double[target.length];
        for (int i = 0; i < target.length; i++) {
            limited[i] = previous[i] + (target[i] - previous[i]) * scale;
        }
        return limited;
    }

    /** Mecanum mixing as {FL, FR, BL, BR}, normalized so no wheel exceeds 1.0. */
    static double[] robotCentricWheelPowers(double axial, double lateral, double yaw) {
        double scaledLateral = lateral * LATERAL_STRAFE_MULTIPLIER;

        double flPower = axial + scaledLateral + yaw;
        double frPower = axial - scaledLateral - yaw;
        double blPower = axial - scaledLateral + yaw;
        double brPower = axial + scaledLateral - yaw;

        double max = Math.max(
                Math.max(Math.abs(flPower), Math.abs(frPower)),
                Math.max(Math.abs(blPower), Math.abs(brPower))
        );
        if (max > 1.0) {
            flPower /= max;
            frPower /= max;
            blPower /= max;
            brPower /= max;
        }

        return new double[] {flPower, frPower, blPower, brPower};
    }

    /**
     * Directly sets normalized wheel powers for individual wheel testing or custom control.
     */
    public void setWheelPowers(double flPower, double frPower, double blPower, double brPower) {
        lastFrontLeftPower = flPower;
        lastFrontRightPower = frPower;
        lastBackLeftPower = blPower;
        lastBackRightPower = brPower;

        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);
    }

    /**
     * Stops all four drivetrain motors.
     */
    public void stop() {
        setWheelPowers(0.0, 0.0, 0.0, 0.0);
    }

    /** Pedro's tuned power model owns velocity control; select once during autonomous INIT. */
    public void useOpenLoopPower() {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Converts direction-adjusted SDK motor encoder velocity to wheel RPM, preserving its sign. */
    public static double motorTicksPerSecondToWheelRpm(double motorTicksPerSecond) {
        return motorTicksPerSecond * 60 / WHEEL_TICKS_PER_REV;
    }

    /** Adds the shared transmission specification without flushing telemetry. */
    public void addConfigurationTelemetry(Telemetry telemetry) {
        telemetry.addData("Drive motor (ticks/rev, RPM)", "%.0f | %.0f no-load @ 12 V",
                BARE_MOTOR_TICKS_PER_REV, BARE_MOTOR_MAX_RPM);
        telemetry.addData("Drive reduction (motor rev/wheel rev)", "%d:%d gears x %d:%d belt = %.5f:1",
                MOTOR_GEAR_TEETH, INTERMEDIATE_GEAR_TEETH, INTERMEDIATE_PULLEY_TEETH,
                WHEEL_PULLEY_TEETH, MOTOR_REVS_PER_WHEEL_REV);
        telemetry.addData("Drive wheel (mm diameter, ticks/rev, RPM)", "%.0f | %.3f | %.1f no-load estimate",
                WHEEL_DIAMETER_MM, WHEEL_TICKS_PER_REV, WHEEL_MAX_RPM);
    }

    /**
     * Adds drivetrain power, encoder position, and velocity data to telemetry.
     * Does not call telemetry.update() so the caller OpMode can flush telemetry once per loop.
     */
    public void addTelemetry(Telemetry telemetry) {
        addMotorTelemetry(telemetry, "FL", frontLeft, lastFrontLeftPower);
        addMotorTelemetry(telemetry, "FR", frontRight, lastFrontRightPower);
        addMotorTelemetry(telemetry, "BL", backLeft, lastBackLeftPower);
        addMotorTelemetry(telemetry, "BR", backRight, lastBackRightPower);
    }

    private void addMotorTelemetry(Telemetry telemetry, String label, DcMotorEx motor, double power) {
        double motorTicksPerSecond = motor.getVelocity();
        telemetry.addData(label, "pwr %+.2f | motor pos %d ticks | motor vel %.0f ticks/s | wheel %.1f RPM",
                power, motor.getCurrentPosition(), motorTicksPerSecond,
                motorTicksPerSecondToWheelRpm(motorTicksPerSecond));
    }

    private void configureBareMotor(DcMotorEx motor, DcMotor.Direction direction) {
        motor.setPower(0);
        motor.setDirection(direction);

        // The SDK describes the bare motor shaft (28 ticks/rev, 5800 RPM, internal gearing 1:1).
        // Applying external reduction here would count it twice in wheel conversions.
        // Clone to avoid mutating the shared motor type singleton.
        MotorConfigurationType motorType = motor.getMotorType().clone();
        motorType.setTicksPerRev(BARE_MOTOR_TICKS_PER_REV);
        motorType.setMaxRPM(BARE_MOTOR_MAX_RPM);
        motorType.setAchieveableMaxRPMFraction(1.0);
        motorType.setGearing(1.0);
        motor.setMotorType(motorType);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private DcMotorEx getDriveMotor(HardwareMap hwMap, String... candidateNames) {
        for (String name : candidateNames) {
            DcMotorEx motor = hwMap.tryGet(DcMotorEx.class, name);
            if (motor != null) {
                return motor;
            }
        }
        return hwMap.get(DcMotorEx.class, candidateNames[0]);
    }
}
