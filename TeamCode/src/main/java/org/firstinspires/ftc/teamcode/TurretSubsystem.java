package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

/**
 * Layer 1 Subsystem: Turret & Shooter Assembly with Limelight 3A Auto-Aiming.
 *
 * <p>Hardware Architecture (Expansion Hub + Control Hub USB 3.0):
 * <ul>
 *     <li><b>Turret Rotation Servos (2x Axon MAX+, Servo Mode, 1:1 with turret)</b>:
 *         Expansion Hub Servo Ports 1 & 2 (powered via goBILDA 6V Servo Power Injector 3125-0001-0001).
 *         Config names: {@code "turret_servo_1"} and {@code "turret_servo_2"}.</li>
 *     <li><b>Turret Rotation Encoder (REV Through-Bore Encoder V1, REV-11-1271)</b>:
 *         Expansion Hub Quadrature Port 1 (8192 CPR, geared 108T Turret : 40T Encoder = 2.7x faster).
 *         Read via the intake motor's {@code DcMotorEx} encoder channel (the intake has no encoder).
 *         Config name: {@code "intake_motor"}; standalone alias {@code "turret_encoder"}.</li>
 *     <li><b>Turret Pitch / Hood Servo (1x Axon MAX+, Servo Mode, 2:1 linkage with hood angle)</b>:
 *         Expansion Hub Servo Port 0 (powered via goBILDA 6V Servo Power Injector 3125-0001-0001).
 *         Home is {@code PITCH_HOME_SERVO_POS}, the team's +15 deg launch reference.
 *         Increasing PWM lowers launch angle to -15 deg (30 deg hood travel), away from the upper stop.
 *         Config name: {@code "turret_pitch"}.</li>
 *     <li><b>Turret Shooter / Flywheel Motor (goBILDA 5203 Yellow Jacket 6000 RPM, 1:1 to flywheel)</b>:
 *         Expansion Hub Motor Port 0 & Quadrature Port 0 (28 ticks/rev, 6000 RPM, FLOAT).
 *         Config name: {@code "turret_motor"}.</li>
 *     <li><b>Limelight 3A Vision Sensor</b>:
 *         Control Hub USB 3.0 port (via powered USB hub), pitched {@link ShootingConfiguration#CAMERA_PITCH_UP_DEG} above horizontal.
 *         Config name: {@code "limelight"}.</li>
 * </ul>
 */
public class TurretSubsystem {

    // =========================================================================
    // HARDWARE MAP CONFIGURATION NAMES
    // =========================================================================
    public static final String HW_ROTATION_SERVO_1 = "turret_servo_1"; // Exp Hub Servo Port 1
    public static final String HW_ROTATION_SERVO_2 = "turret_servo_2"; // Exp Hub Servo Port 2
    public static final String HW_PITCH_SERVO      = "turret_pitch";   // Exp Hub Servo Port 0
    public static final int ROTATION_SERVO_1_PORT = 1;
    public static final int ROTATION_SERVO_2_PORT = 2;
    public static final int PITCH_SERVO_PORT = 0;
    public static final String HW_TURRET_MOTOR     = "turret_motor";   // Exp Hub Motor & Quad Port 0
    public static final String HW_TURRET_ENCODER   = "turret_encoder"; // Standalone alias for Exp Hub Quad Port 1
    public static final String HW_LIMELIGHT        = "limelight";      // Control Hub USB 3.0

    // =========================================================================
    // AXON MAX+ SERVO PULSE WIDTH & RANGE CONSTANTS
    // =========================================================================
    public static final int AXON_MIN_PULSE_US = AxonMaxPlus.MIN_PULSE_US;
    public static final int AXON_MAX_PULSE_US = AxonMaxPlus.MAX_PULSE_US;
    public static final double AXON_MAX_TRAVEL_DEG = AxonMaxPlus.PROGRAMMED_TRAVEL_DEG; // Programmed travel across 0.0..1.0

    // =========================================================================
    // TURRET ROTATION (YAW) & DEADZONE CONSTANTS (1:1 GEARING)
    // =========================================================================
    // Robot headings are clockwise: front=0, right=90, back=180, left=270 deg.
    // Center (0.5) is home at robot heading 90 deg, internal servo-relative yaw 0 deg.
    // Internal yaw stays counterclockwise for field geometry; it is NOT a robot heading.
    // Deadzone of +/-5 deg at the left (robot headings 265..275, internal yaw +/-180) prevents wire wrap
    // and keeps the servos safely away from the 0.0 / 1.0 PWM discontinuity.
    public static final double ROTATION_GEAR_RATIO = 1.0; // 1:1 servo-to-turret rotation
    public static final double ROTATION_HOME_SERVO_POS = 0.5;
    public static final double ROTATION_HOME_ROBOT_HEADING_DEG = 90.0; // Single source for TurretControl headings
    public static final double ROTATION_HOME_ANGLE_DEG = 0.0;
    public static final double ROTATION_DEADZONE_HALF_DEG = 5.0; // +/- 5 deg deadzone on the left
    public static final double ROTATION_MIN_ANGLE_DEG = -180.0 + ROTATION_DEADZONE_HALF_DEG; // -175.0 deg
    public static final double ROTATION_MAX_ANGLE_DEG =  180.0 - ROTATION_DEADZONE_HALF_DEG; // +175.0 deg
    // While the turret sits near the back on one side, a target just past the dead zone on the other side
    // must clear this margin before the turret unwinds the long way, so edge noise cannot cause repeat spins.
    public static final double ROTATION_WRAP_HYSTERESIS_DEG = 3.0;

    // Set to -1.0 if positive angle commands move the turret in the opposite direction of encoder ticks
    public static final double ROTATION_SERVO_DIRECTION_SIGN = -1.0;

    // Both rotation servos mesh with the turret ring in the same orientation, so they share ONE direction
    // (and one PWM range and one position command) through PairedServos. There is intentionally no
    // per-servo direction: opposite directions on a shared gear strip teeth. To reverse the turret,
    // change ROTATION_SERVO_DIRECTION_SIGN or this one constant, never one servo alone.
    public static final Servo.Direction ROTATION_SERVO_DIRECTION = Servo.Direction.FORWARD;
    static final String[] ROTATION_SERVO_1_NAMES = {HW_ROTATION_SERVO_1, "turretServo1", "turret_left", "turret1"};
    static final String[] ROTATION_SERVO_2_NAMES = {HW_ROTATION_SERVO_2, "turretServo2", "turret_right", "turret2"};

    // =========================================================================
    // REV THROUGH-BORE ENCODER V1 (QUAD PORT 1 VIA intake_motor, 108T TURRET : 40T ENCODER)
    // =========================================================================
    // REV-11-1271 quadrature resolution = 8192 counts per revolution (2048 CPR * 4x).
    // Geared 108T on turret/servos : 40T on encoder => encoder spins 2.7x faster than turret.
    public static final double THROUGH_BORE_CPR = 8192.0;
    public static final double TURRET_TO_ENCODER_GEAR_RATIO = 108.0 / 40.0; // 2.7
    public static final double TURRET_TICKS_PER_REV = THROUGH_BORE_CPR * TURRET_TO_ENCODER_GEAR_RATIO; // 22,118.4 ticks/rev
    public static final double TURRET_TICKS_PER_DEGREE = TURRET_TICKS_PER_REV / 360.0; // 61.44 ticks/deg
    public static final double ENCODER_DIRECTION_SIGN = 1.0; // Flip to -1.0 if encoder counts decrease on +angle

    // Internal servo feedback handles normal positioning; external correction starts above the margin.
    public static final double SERVO_CUTOUT_ERROR_THRESHOLD_DEG = TurretControl.ENCODER_CORRECTION_MARGIN_DEG;

    // =========================================================================
    // HOOD TRAVEL FROM HOME (PORT 0, 2:1 LINKAGE): 0..30 DEG => LAUNCH +15..-15 DEG
    // =========================================================================
    // Home includes a 10.5-degree downward offset from the original 0.5 mechanical reference.
    // Linked 2:1 with turret angle: 1 deg of turret pitch requires 2 deg of Axon MAX+ servo rotation
    // (or set PITCH_SERVO_TO_TURRET_RATIO = 1.0 if measuring servo shaft degrees directly).
    // Increasing PWM moves down. Never command above the new home toward the upper stop.
    public static final double PITCH_MIN_DEG = 0.0;
    public static final double PITCH_MAX_DEG = 30.0;
    public static final double LAUNCH_HOME_ANGLE_DEG = 15.0;
    public static final double PITCH_HOME_DEG = 0.0;
    public static final double PITCH_SERVO_TO_TURRET_RATIO = 2.0; // 60 deg servo travel = 30 deg hood travel
    // Previous +12.5-degree position (2.5 degrees below the 8-degree-offset home) is the new +15 reference.
    public static final double PITCH_HOME_OFFSET_DEG = 10.5;
    public static final double PITCH_HOME_SERVO_POS = 0.5
            + PITCH_HOME_OFFSET_DEG * PITCH_SERVO_TO_TURRET_RATIO / AXON_MAX_TRAVEL_DEG;
    public static final double PITCH_SERVO_POS_AT_0_DEG = PITCH_HOME_SERVO_POS;
    public static final double PITCH_SERVO_DEG_PER_UNIT = AXON_MAX_TRAVEL_DEG; // 355.0 deg across 0.0..1.0
    public static final double PITCH_SERVO_DIRECTION_SIGN = 1.0;
    public static final Servo.Direction PITCH_SERVO_DIRECTION = Servo.Direction.FORWARD;

    // =========================================================================
    // TURRET SHOOTER / FLYWHEEL MOTOR CONSTANTS (PORT 0, 6000 RPM BARE MOTOR)
    // =========================================================================
    public static final double SHOOTER_MOTOR_TICKS_PER_REV = ShooterSpeed.MOTOR_TICKS_PER_REV;
    public static final double SHOOTER_MOTOR_MAX_RPM = ShooterSpeed.MOTOR_MAX_RPM;
    public static final double SHOOTER_FLYWHEEL_REVS_PER_MOTOR_REV = ShooterSpeed.FLYWHEEL_REVS_PER_MOTOR_REV;
    public static final double SHOOTER_MAX_RPM = ShooterSpeed.FLYWHEEL_MAX_TARGET_RPM;
    // Presets and readiness tolerances are output-shaft RPM, independent of motor gearing.
    public static final double SHOOTER_DEFAULT_TARGET_RPM = 2800.0;
    public static final double SHOOTER_MIN_AUTO_RPM = 2133.3333333333335;
    public static final double SHOOTER_MAX_AUTO_RPM = SHOOTER_MAX_RPM;
    public static final double SHOOTER_READY_TOLERANCE_RPM = 100.0;
    public static final double SHOOTER_LOOSE_READY_TOLERANCE_RPM = 133.33333333333334;
    public static final double SHOT_READY_YAW_TOLERANCE_DEG = ShotReadiness.YAW_TOLERANCE_DEG;
    public static final double YAW_VERIFY_MAX_VELOCITY_DEG_PER_SECOND = 3.0;
    public static final double SHOT_READY_PITCH_TOLERANCE_DEG = 0.25;
    public static final double HOOD_SETTLE_TIME_SECONDS = 0.3;

    // =========================================================================
    // LIMELIGHT 3A MOUNTING & BIOBUZZ HIVE / APRILTAG FIELD CONSTANTS
    // =========================================================================
    // Single source: the shooter mount model's camera pitch (the old 30 deg value here was stale).
    public static final double LIMELIGHT_MOUNT_ANGLE_DEG = ShootingConfiguration.CAMERA_PITCH_UP_DEG;
    public static final double LIMELIGHT_LENS_HEIGHT_MM = ShootingConfiguration.CAMERA_HEIGHT_MM;
    public static final double LIMELIGHT_LENS_HEIGHT_INCHES = LIMELIGHT_LENS_HEIGHT_MM / 25.4;
    public static final int LIMELIGHT_DEFAULT_PIPELINE = 0;

    // Robot-frame turret pivot offset; the rotating camera offset is separate in ShootingConfiguration.
    public static final double TURRET_OFFSET_FORWARD_IN = ShootingConfiguration.TURRET_PIVOT_FORWARD_MM / 25.4;
    public static final double TURRET_OFFSET_LEFT_IN = ShootingConfiguration.TURRET_PIVOT_LEFT_MM / 25.4;

    // Math uses CCW radians: right-facing home is -90 deg (clockwise robot heading 90 deg).
    // Single source for ShootingConfiguration.MOUNT.zeroYaw; must equal -ROTATION_HOME_ROBOT_HEADING_DEG (mod 360).
    public static final double TURRET_ZERO_ROBOT_HEADING_OFFSET_DEG = -90.0;

    // TU06 drawing dimensions share one source; derived field positions remain nominal.
    // Team planning field frame (must match route and tag survey): (0, 0) at field center,
    // +X toward Blue alliance (right), -X toward Red alliance (left),
    // +Y toward Far / Scoring Table side, -Y toward Audience side.
    public static final double FIELD_SIZE_INCHES = FieldGeometry.FIELD_SIZE_INCHES;
    public static final double HIVE_CENTER_TO_CENTER_INCHES = FieldGeometry.HIVE_CENTER_SPACING_INCHES;
    public static final double RED_HIVE_PIVOT_X_IN = -HIVE_CENTER_TO_CENTER_INCHES / 2.0;  // -12.75 in
    public static final double BLUE_HIVE_PIVOT_X_IN = +HIVE_CENTER_TO_CENTER_INCHES / 2.0; // +12.75 in
    public static final double HIVE_PIVOT_Y_IN = 0.0;
    public static final double HIVE_PIVOT_Z_IN = FieldGeometry.HIVE_PIVOT_HEIGHT_INCHES;
    public static final double HIVE_TILT_DEG = FieldGeometry.HIVE_TILT_DEG;
    public static final double HIVE_CELL_GAP_IN = FieldGeometry.HIVE_CELL_GAP_INCHES;
    public static final double HIVE_CELL_DEPTH_IN = FieldGeometry.HIVE_CELL_DEPTH_INCHES;
    public static final double HIVE_OPENING_WIDTH_IN = FieldGeometry.OPENING_WIDTH_INCHES;
    public static final double HIVE_OPENING_SHOULDER_HEIGHT_IN = FieldGeometry.OPENING_SHOULDER_HEIGHT_INCHES;
    public static final double HIVE_OPENING_PEAK_HEIGHT_IN = FieldGeometry.OPENING_PEAK_HEIGHT_INCHES;
    public static final double HIVE_MOUTH_BOTTOM_Z_IN = FieldGeometry.RAISED_OPENING_BOTTOM_HEIGHT_INCHES;

    // Team-approved drawing estimate; do not infer localization tag coordinates from it.
    public static final double HIVE_RAISED_MOUTH_CENTER_Y_OFFSET_IN = TeleOpFieldReference.OPENING_Y_OFFSET_METERS / .0254;
    public static final double HIVE_RAISED_MOUTH_CENTER_Z_IN = ShootingConfiguration.HIGH_GOAL_CENTER_HEIGHT_METERS / .0254;
    public static final double GOAL_TARGET_HEIGHT_INCHES = HIVE_RAISED_MOUTH_CENTER_Z_IN;

    // BIOBUZZ AprilTag IDs on each Hive Cell (4-tag clusters on the underside of each cell):
    // Red Far (Scoring Side): 30..33 | Red Audience: 34..37
    // Blue Audience: 38..41          | Blue Far (Scoring Side): 42..45
    public static final int RED_FAR_TAG_MIN = 30;
    public static final int RED_FAR_TAG_MAX = 33;
    public static final int RED_AUDIENCE_TAG_MIN = 34;
    public static final int RED_AUDIENCE_TAG_MAX = 37;
    public static final int BLUE_AUDIENCE_TAG_MIN = 38;
    public static final int BLUE_AUDIENCE_TAG_MAX = 41;
    public static final int BLUE_FAR_TAG_MIN = 42;
    public static final int BLUE_FAR_TAG_MAX = 45;

    public enum Alliance {
        RED,
        BLUE
    }

    public enum HiveSide {
        FAR,      // Scoring table side (+Y): Tags 30-33 (Red) or 42-45 (Blue)
        AUDIENCE  // Audience side (-Y): Tags 34-37 (Red) or 38-41 (Blue)
    }

    public static final long LIMELIGHT_MAX_STALENESS_MS = LimelightTagSubsystem.MAX_STALENESS_MS;
    public static final double HOME_TRAVEL_SECONDS = 1.5, HOME_STABLE_SECONDS = .35;
    public static final double HOME_TIMEOUT_SECONDS = 5, HOME_STABLE_WINDOW_DEG = .3;

    // =========================================================================
    // HARDWARE DEVICES
    // =========================================================================
    private final PairedServos rotationServos;
    private final Servo pitchServo;
    private final DcMotorEx shooterMotor;
    private final DcMotorEx throughBoreEncoderPort;
    private final Limelight3A limelight;
    private final boolean visionEnabled;
    private final java.util.List<VoltageSensor> voltageSensors = new java.util.ArrayList<>();
    private double batteryVolts = Double.NaN, shooterCurrentAmps = Double.NaN;
    /** PIDF is the subsystem default; competition TeleOps and the combined bench test can select fusion. */
    public enum ShooterController { PIDF, BANG_BANG_FUSION }
    private ShooterController shooterController = ShooterController.PIDF;
    private double fusionPower, shooterTicksPerSecond;
    private boolean fusionBangBang;
    private double fusionKs = ShooterVelocityTuning.INITIAL_FUSION_KS;
    private double fusionKv = ShooterVelocityTuning.INITIAL_FUSION_KV;
    private double fusionKp = ShooterVelocityTuning.INITIAL_FUSION_KP;
    private double shooterP = ShooterVelocityTuning.INITIAL_P, shooterFScale = ShooterVelocityTuning.INITIAL_F_SCALE;
    private double shooterI = ShooterVelocityTuning.INITIAL_I;
    private double shooterD = ShooterVelocityTuning.INITIAL_D;
    private double appliedShooterP = Double.NaN, appliedShooterI = Double.NaN, appliedShooterD = Double.NaN;
    private double appliedShooterF, lastShooterSample = Double.NEGATIVE_INFINITY;

    private final TurretHomingController homing = new TurretHomingController(HOME_TRAVEL_SECONDS, HOME_STABLE_SECONDS, HOME_TIMEOUT_SECONDS, HOME_STABLE_WINDOW_DEG);
    private final TurretControl control = new TurretControl();
    private int rawTicks, encoderZeroOffsetTicks;
    private double encoderVelocity, currentRpm, targetYawDeg, targetPitchDeg, targetShooterRpm;
    private double peakTurretRateDegPerSecond;
    private double lastCommandedRotationServoPos = .5, lastCommandedPitchServoPos = .5;
    private Alliance activeAlliance = Alliance.RED;
    private HiveSide activeHiveSide = HiveSide.FAR;
    private LLResult cachedResult;
    private final LimelightObservation.FrameRate visionFrameRate = new LimelightObservation.FrameRate();
    private LimelightObservation cachedObservation;
    private LLResultTypes.FiducialResult cachedTag;
    private String trackingStatus = "Automatic aiming requires ShotCoordinator";
    private final java.util.function.DoubleSupplier clock;

    public TurretSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, true);
    }

    /** Manual mechanism diagnostics can omit the camera entirely. */
    public TurretSubsystem(HardwareMap hardwareMap, boolean enableVision) {
        this(hardwareMap, enableVision, () -> System.nanoTime() * 1e-9);
    }
    public TurretSubsystem(HardwareMap hardwareMap, boolean enableVision, java.util.function.DoubleSupplier clock) {
        this.clock = clock;
        visionEnabled = enableVision;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) voltageSensors.add(sensor);
        // Configured (one direction, one PWM range) and verified matched before any position is written.
        rotationServos = new PairedServos("Turret rotation servos",
                getTurretServo(hardwareMap, ROTATION_SERVO_1_PORT, ROTATION_SERVO_1_NAMES),
                getTurretServo(hardwareMap, ROTATION_SERVO_2_PORT, ROTATION_SERVO_2_NAMES),
                ROTATION_SERVO_DIRECTION, AXON_MIN_PULSE_US, AXON_MAX_PULSE_US);
        pitchServo = getTurretServo(hardwareMap, PITCH_SERVO_PORT,
                HW_PITCH_SERVO, "turretPitch", "hood_servo", "pitch_servo");
        shooterMotor = NamedHardware.requireMotor(hardwareMap,
                HW_TURRET_MOTOR, "turretMotor", "shooter_motor", "shooter", "flywheel");
        throughBoreEncoderPort = NamedHardware.requireMotor(hardwareMap,
                IntakeSubsystem.HW_INTAKE_MOTOR,
                HW_TURRET_ENCODER, "turretEncoder", "through_bore", "turret_enc");
        limelight = enableVision ? tryGetDevice(hardwareMap, Limelight3A.class,
                HW_LIMELIGHT, "limelight3a") : null;

        try {
            configureServos();
            configureShooterMotor();
            // Intake owns the shared channel's mode/direction; do not configure/reset it here.
            configureLimelight();

            // Begin travel now; update during INIT and after an early START.
            homeTurretAndZeroEncoder();
            setPitchAngleDegrees(PITCH_HOME_DEG);
        } catch (RuntimeException failure) {
            try { stop(); } catch (RuntimeException cleanup) { failure.addSuppressed(cleanup); }
            throw failure;
        }
    }

    // =========================================================================
    // INITIALIZATION & HOMING
    // =========================================================================

    private void configureServos() {
        // The rotation pair was configured and verified by PairedServos.
        configureAxonServo(pitchServo, PITCH_SERVO_DIRECTION);
    }

    private void configureAxonServo(Servo servo, Servo.Direction direction) {
        servo.setDirection(direction);
        if (servo instanceof PwmControl) {
            ((PwmControl) servo).setPwmRange(
                    new PwmControl.PwmRange(AXON_MIN_PULSE_US, AXON_MAX_PULSE_US));
        }
    }

    private void configureShooterMotor() {
        shooterMotor.setPower(0.0);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);

        MotorConfigurationType motorType = shooterMotor.getMotorType().clone();
        motorType.setTicksPerRev(SHOOTER_MOTOR_TICKS_PER_REV);
        motorType.setMaxRPM(SHOOTER_MOTOR_MAX_RPM);
        motorType.setAchieveableMaxRPMFraction(1.0);
        motorType.setGearing(1.0);
        shooterMotor.setMotorType(motorType);

        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Flywheels should coast (FLOAT) rather than brake abruptly
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        updateShooterPidf(clock.getAsDouble(), true);
    }

    private void updateShooterPidf(double now, boolean force) {
        if (!force && now - lastShooterSample < ShooterVelocityTuning.SAMPLE_INTERVAL_SECONDS) return;
        lastShooterSample = now;
        batteryVolts = Double.NaN;
        for (VoltageSensor sensor : voltageSensors) {
            double volts = sensor.getVoltage();
            if (Double.isFinite(volts) && volts >= 9 && volts <= 16
                    && (!Double.isFinite(batteryVolts) || volts < batteryVolts)) batteryVolts = volts;
        }
        shooterCurrentAmps = shooterMotor.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
        double nextF = ShooterVelocityTuning.feedforward(batteryVolts, shooterFScale);
        // The hub runs feedback continuously. Avoid rewriting unchanged gains for every sensor sample.
        if (shooterController == ShooterController.PIDF && (force || shooterP != appliedShooterP || shooterI != appliedShooterI || shooterD != appliedShooterD
                || Math.abs(nextF - appliedShooterF) >= ShooterVelocityTuning.MIN_F_CHANGE)) {
            shooterMotor.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, nextF);
            appliedShooterP = shooterP; appliedShooterI = shooterI; appliedShooterD = shooterD; appliedShooterF = nextF;
        }
    }
    public void adjustShooterVelocityTuning(double deltaP, double deltaFScale) {
        adjustShooterVelocityTuning(deltaP, 0, deltaFScale);
    }
    public void adjustShooterVelocityTuning(double deltaP, double deltaI, double deltaFScale) {
        if (!RobotMath.finite(deltaP, deltaI, deltaFScale)) return;
        setShooterVelocityTuning(shooterP + deltaP, shooterI + deltaI, shooterD, shooterFScale + deltaFScale);
    }
    /** Absolute live settings for bench tuning. Unchanged settings cause no hub writes or sensor reads. */
    public void setShooterVelocityTuning(double p, double i, double d, double fScale) {
        if (!RobotMath.finite(p, i, d, fScale)) return;
        p = Math.max(0, Math.min(100, p));
        i = Math.max(0, Math.min(ShooterVelocityTuning.MAX_I, i));
        d = Math.max(0, Math.min(100, d));
        fScale = Math.max(.5, Math.min(1.5, fScale));
        if (p == shooterP && i == shooterI && d == shooterD && fScale == shooterFScale) return;
        shooterP = p; shooterI = i; shooterD = d; shooterFScale = fScale;
        updateShooterPidf(clock.getAsDouble(), true);
    }
    public void resetShooterVelocityTuning() {
        setShooterVelocityTuning(ShooterVelocityTuning.INITIAL_P, ShooterVelocityTuning.INITIAL_I,
                ShooterVelocityTuning.INITIAL_D, ShooterVelocityTuning.INITIAL_F_SCALE);
    }
    public double getShooterP() { return shooterP; }
    public double getShooterI() { return shooterI; }
    public double getShooterD() { return shooterD; }
    public double getShooterFScale() { return shooterFScale; }
    public double getShooterAppliedF() { return appliedShooterF; }
    /** Cached electrical readings sampled at 5 Hz, not additional hardware reads. */
    public double getShooterBatteryVolts() { return batteryVolts; }
    public double getShooterCurrentAmps() { return shooterCurrentAmps; }

    private void configureLimelight() {
        if (limelight != null) {
            limelight.pipelineSwitch(LIMELIGHT_DEFAULT_PIPELINE);
            limelight.start();
        }
    }

    /** Begins non-blocking estimated homing; zero is captured only after travel and stability. */
    public void homeTurretAndZeroEncoder() {
        homeTurretAndZeroEncoder(clock.getAsDouble());
    }
    public void homeTurretAndZeroEncoder(double now) {
        targetYawDeg = 0; control.reset();
        homing.start(now);
        setRawRotationServoPosition(ROTATION_HOME_SERVO_POS);
    }
    /** Diagnostic relative zero deliberately invalidates automatic firing until rehomed. */
    public void zeroEncoderAtCurrentPosition() {
        encoderZeroOffsetTicks = rawTicks; control.reset(); homing.invalidate();
    }
    public void update() {
        update(clock.getAsDouble());
    }
    public void update(double now) {
        updateShooterPidf(now, false);
        // Undo the SDK motor-direction sign of the shared intake channel so yaw uses raw quadrature counts.
        int encoderPortSign = throughBoreEncoderPort.getDirection() == DcMotor.Direction.REVERSE ? -1 : 1;
        rawTicks = throughBoreEncoderPort.getCurrentPosition() * encoderPortSign;
        encoderVelocity = throughBoreEncoderPort.getVelocity() * encoderPortSign
                / TURRET_TICKS_PER_DEGREE * ENCODER_DIRECTION_SIGN;
        shooterTicksPerSecond = shooterMotor.getVelocity();
        currentRpm = ShooterSpeed.flywheelRpm(shooterTicksPerSecond);
        if (Double.isFinite(encoderVelocity)) peakTurretRateDegPerSecond = Math.max(peakTurretRateDegPerSecond, Math.abs(encoderVelocity));
        boolean wasReady = homing.ready();
        homing.update(now, rawTicks, rawTicks / TURRET_TICKS_PER_DEGREE * ENCODER_DIRECTION_SIGN);
        if (!wasReady && homing.ready()) encoderZeroOffsetTicks = homing.zeroTicks();
        // Hood travel estimate covers the interval the previous loop's command was in effect.
        control.updateHoodEstimate(now, targetPitchDeg);
        cachedResult = limelight == null ? null : limelight.getLatestResult();
        cachedObservation = visionEnabled ? new LimelightObservation(now,System.currentTimeMillis(),cachedResult,
                visionFrameRate.update(cachedResult,now)) : null;
        cachedTag = null;
        if (cachedResult != null && cachedResult.isValid() && cachedResult.getStaleness() >= 0
                && cachedResult.getStaleness() <= LIMELIGHT_MAX_STALENESS_MS) {
            for (LLResultTypes.FiducialResult tag : cachedResult.getFiducialResults()) {
                if (tag != null && getHiveSideForTag(activeAlliance, tag.getFiducialId()) == activeHiveSide
                        && Double.isFinite(tag.getTargetXDegrees()) && Double.isFinite(tag.getTargetYDegrees())
                        && (cachedTag == null || tag.getFiducialId() < cachedTag.getFiducialId())) cachedTag = tag;
            }
        }
    }
    /**
     * Writes this loop's yaw target to the rotation servos. Call once per loop AFTER update() and after every
     * yaw setter (auto aim, manual stick, hold), so the servo never receives the previous loop's target.
     * update() only reads sensors; a target set after it waited a full loop before this split.
     */
    public void applyYawCommand() { applyYawCommand(clock.getAsDouble()); }
    public void applyYawCommand(double now) {
        double commandedYawDeg = control.updateYawCommand(now, targetYawDeg, getEncoderAngleDegrees(),
                encoderVelocity, homing.ready());
        if (homing.ready()) {
            setRawRotationServoPosition(angleDegreesToRotationServoPos(commandedYawDeg));
        } else if (homing.commandsHome()) {
            setRawRotationServoPosition(ROTATION_HOME_SERVO_POS);
        } // UNHOMED / FAULT retain the last bounded command; only explicit home moves home.
    }
    /** Highest measured turret rate since construction or reset: the loaded servo speed under full manual stick. */
    public double getPeakTurretRateDegPerSec() { return peakTurretRateDegPerSecond; }
    public void resetPeakTurretRate() { peakTurretRateDegPerSecond = 0; }
    public boolean isHomed() { return homing.ready(); }
    public boolean hasHomingFault() { return homing.state() == TurretHomingController.State.FAULT; }
    /** Stops normal travel without losing a valid reference. Never resumes an interrupted home. */
    public void holdYaw() {
        if (homing.ready()) {
            targetYawDeg = clampYawAngle(getEncoderAngleDegrees());
            setRawRotationServoPosition(angleDegreesToRotationServoPos(targetYawDeg));
        } else homing.invalidate(); // Retain last bounded servo command when reference is unknown.
    }
    public boolean hasLimelight() { return limelight != null; }
    /** Enable only after checking encoder sign and physical angle against telemetry. */
    public void setEncoderCorrectionEnabled(boolean enabled) { control.setEncoderCorrectionEnabled(enabled); }
    public boolean isEncoderCorrectionEnabled() { return control.isEncoderCorrectionEnabled(); }
    public String getHomingStatus() { return homing.state() + ": " + homing.reason(); }
    public LLResult getVisionResult() { return cachedResult; }
    public LimelightObservation getVisionObservation() { return cachedObservation; }
    public LLResultTypes.FiducialResult getVisionTag() { return cachedTag; }
    public void setTrackingStatus(String status) { trackingStatus = status; }
    public void resetTrackingTrim() { /* Corrections reset on target changes, never on button release. */ }
    public void setTargetYawDegrees(double deg) { if (Double.isFinite(deg)) targetYawDeg = clampYawAngle(deg); }
    public void adjustTargetYawDegrees(double delta) { setTargetYawDegrees(targetYawDeg + delta); }
    /** GP1 auto-aim owns yaw while held; release holds the last commanded target. */
    public void updateManualYaw(double stickX, double dt, boolean automatic) {
        if (!isHomed()) return;
        setTargetYawDegrees(control.manualYawTarget(targetYawDeg, stickX, dt, automatic));
    }
    /** Competition release captures measured yaw once; diagnostic test controls remain independent. */
    public void updateCompetitionManualYaw(double stickX, double dt, boolean automatic) {
        if (!isHomed()) return;
        setTargetYawDegrees(control.competitionManualYawTarget(targetYawDeg, getEncoderAngleDegrees(), stickX, dt, automatic));
    }
    public double getRobotHeadingDegrees() { return TurretControl.yawToRobotHeadingDegrees(getEncoderAngleDegrees()); }
    public double getTargetRobotHeadingDegrees() { return TurretControl.yawToRobotHeadingDegrees(targetYawDeg); }
    public double getTargetYawDegrees() { return targetYawDeg; }
    public int getEncoderTicks() { return rawTicks - encoderZeroOffsetTicks; }
    public double getEncoderAngleDegrees() { return getEncoderTicks() / TURRET_TICKS_PER_DEGREE * ENCODER_DIRECTION_SIGN; }
    public double getEncoderVelocityDegPerSec() { return encoderVelocity; }
    public boolean isRotationErrorHigh() { return Math.abs(targetYawDeg - getEncoderAngleDegrees()) > SERVO_CUTOUT_ERROR_THRESHOLD_DEG; }
    public boolean isYawPositionVerified() {
        return isHomed() && Math.abs(targetYawDeg - getEncoderAngleDegrees()) <= SHOT_READY_YAW_TOLERANCE_DEG
                && Math.abs(encoderVelocity) <= YAW_VERIFY_MAX_VELOCITY_DEG_PER_SECOND;
    }
    public void setRawRotationServoPosition(double pos) {
        if (!Double.isFinite(pos)) return;
        lastCommandedRotationServoPos = clampRotationServoPosition(pos);
        rotationServos.setPosition(lastCommandedRotationServoPos); // Always both servos, same position.
    }
    public double getLastCommandedRotationServoPos() { return lastCommandedRotationServoPos; }
    public static double clampRotationServoPosition(double pos) {
        double a = angleDegreesToRotationServoPos(ROTATION_MIN_ANGLE_DEG);
        double b = angleDegreesToRotationServoPos(ROTATION_MAX_ANGLE_DEG);
        return Range.clip(pos, Math.min(a, b), Math.max(a, b));
    }
    /**
     * Shared automatic-aim yaw rule. Any desired bearing (any wrap) becomes a reachable target.
     * Inside the rear +/-5-deg dead zone: with the turret already in its rear half, hold the limit on
     * that side; from the front half (including mid-unwind), take the limit nearest the bearing, so the
     * turret ends closest to the goal and does not reverse an unwind. Once the bearing leaves the zone on
     * the other side by the hysteresis margin, the target moves there and the turret unwinds the long way
     * through home instead of crossing the dead zone.
     *
     * <p>Pass the previous yaw TARGET as {@code currentDeg} (see {@link #reachableYawTarget}), not the measured
     * yaw. The target is the rule's memory: with the measured yaw, a bearing jittering across the back
     * flipped the target between +175 and -175 every loop while the turret was still in its front half,
     * and noise at the hysteresis edge reversed an unwind mid-travel, so the turret jerked back and forth.
     */
    public static double reachableYawDegrees(double desiredDeg, double currentDeg) {
        if (!Double.isFinite(desiredDeg)) return Double.NaN;
        double desired = normalizeAngleDegrees(desiredDeg);
        boolean nearBack = Double.isFinite(currentDeg) && Math.abs(currentDeg) > 90;
        double side = nearBack && currentDeg < 0 ? -1 : 1;
        if (Math.abs(desired) > ROTATION_MAX_ANGLE_DEG)
            return (nearBack ? side : desired < 0 ? -1 : 1) * ROTATION_MAX_ANGLE_DEG;
        if (nearBack && Math.signum(desired) != side && Math.abs(desired) > ROTATION_MAX_ANGLE_DEG - ROTATION_WRAP_HYSTERESIS_DEG)
            return side * ROTATION_MAX_ANGLE_DEG;
        return desired;
    }
    /** {@link #reachableYawDegrees} using this turret's current target as the dead-zone side memory. */
    public double reachableYawTarget(double desiredDeg) { return reachableYawDegrees(desiredDeg, targetYawDeg); }
    public static double clampYawAngle(double deg) { return Range.clip(deg, ROTATION_MIN_ANGLE_DEG, ROTATION_MAX_ANGLE_DEG); }
    public static double angleDegreesToRotationServoPos(double deg) {
        return Range.clip(.5 + deg * ROTATION_GEAR_RATIO * ROTATION_SERVO_DIRECTION_SIGN / AXON_MAX_TRAVEL_DEG, 0, 1);
    }
    /** Commands hood travel down from home (0..30 deg), not elevation above horizontal. */
    public void setPitchAngleDegrees(double deg) {
        if (!Double.isFinite(deg)) return;
        targetPitchDeg = Range.clip(deg, PITCH_MIN_DEG, PITCH_MAX_DEG);
        setRawPitchServoPosition(pitchDegreesToServoPos(targetPitchDeg));
    }
    public void adjustPitchAngleDegrees(double delta) { setPitchAngleDegrees(targetPitchDeg + delta); }
    public double getTargetPitchDegrees() { return targetPitchDeg; }
    /** Commanded geometric exit elevation; this is not a measured projectile trajectory. */
    public static double hoodTravelToLaunchAngleDegrees(double hoodTravelDeg) {
        return LAUNCH_HOME_ANGLE_DEG - Range.clip(hoodTravelDeg, PITCH_MIN_DEG, PITCH_MAX_DEG);
    }
    public void setRawPitchServoPosition(double pos) {
        if (!Double.isFinite(pos)) return;
        lastCommandedPitchServoPos = clampPitchServoPosition(pos);
        pitchServo.setPosition(lastCommandedPitchServoPos);
    }
    public double getLastCommandedPitchServoPos() { return lastCommandedPitchServoPos; }
    public static double clampPitchServoPosition(double pos) {
        double end = pitchDegreesToServoPos(PITCH_MAX_DEG);
        return Range.clip(pos, Math.min(PITCH_HOME_SERVO_POS, end), Math.max(PITCH_HOME_SERVO_POS, end));
    }
    public static double pitchDegreesToServoPos(double deg) {
        return Range.clip(PITCH_HOME_SERVO_POS + PITCH_SERVO_DIRECTION_SIGN * Range.clip(deg, PITCH_MIN_DEG, PITCH_MAX_DEG)
                * PITCH_SERVO_TO_TURRET_RATIO / PITCH_SERVO_DEG_PER_UNIT, 0, 1);
    }
    public boolean isHoodTracking() { return control.hoodError(targetPitchDeg) <= SHOT_READY_PITCH_TOLERANCE_DEG; }
    /** Switching stops the command; the caller must explicitly request spin again. */
    public void setShooterController(ShooterController controller) {
        if (controller == null || controller == shooterController) return;
        stopShooter();
        shooterController = controller;
        shooterMotor.setMode(controller == ShooterController.PIDF
                ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if (controller == ShooterController.PIDF) updateShooterPidf(clock.getAsDouble(), true);
    }
    public ShooterController getShooterController() { return shooterController; }
    /** Bench fusion gains in motor power at 12 V. Software-only: no hub writes; invalid values are ignored. */
    public void setShooterFusionTuning(double kS, double kV, double kP) {
        if (!RobotMath.finite(kS, kV, kP)) return;
        fusionKs = Range.clip(kS, 0, ShooterVelocityTuning.MAX_FUSION_KS);
        fusionKv = Range.clip(kV, 0, ShooterVelocityTuning.MAX_FUSION_KV);
        fusionKp = Range.clip(kP, 0, ShooterVelocityTuning.MAX_FUSION_KP);
    }
    public double getShooterFusionKs() { return fusionKs; }
    public double getShooterFusionKv() { return fusionKv; }
    public double getShooterFusionKp() { return fusionKp; }
    /** Last fusion motor command (-1..1); zero in PIDF mode or while stopped. */
    public double getShooterFusionPower() { return fusionPower; }
    /** True while the last fusion command was bang-bang (full power or coast): error above 80 ticks/s. */
    public boolean isShooterFusionBangBang() { return fusionBangBang; }
    public double getShooterTicksPerSecond() { return shooterTicksPerSecond; }

    /** Call once per loop after update(); the fusion law uses that loop's cached encoder sample. */
    public void setShooterRpm(double rpm) {
        if (!Double.isFinite(rpm)) { stopShooter(); return; }
        targetShooterRpm = ShooterSpeed.clampFlywheelRpm(rpm);
        if (targetShooterRpm == 0) { stopShooter(); return; }
        if (shooterController == ShooterController.BANG_BANG_FUSION) {
            double targetTicksPerSecond = ShooterSpeed.motorTicksPerSecond(targetShooterRpm);
            fusionBangBang = ShooterVelocityTuning.fusionUsesBangBang(targetTicksPerSecond, shooterTicksPerSecond);
            fusionPower = ShooterVelocityTuning.fusionPower(targetTicksPerSecond, shooterTicksPerSecond,
                    batteryVolts, fusionKs, fusionKv, fusionKp);
            shooterMotor.setPower(fusionPower);
        } else shooterMotor.setVelocity(ShooterSpeed.motorTicksPerSecond(targetShooterRpm));
    }
    public void setShooterPower(double power) {
        targetShooterRpm = Range.clip(power, -1, 1) * SHOOTER_MAX_RPM; shooterMotor.setPower(Range.clip(power, -1, 1));
    }
    public void stopShooter() { targetShooterRpm = 0; fusionPower = 0; fusionBangBang = false; shooterMotor.setPower(0); }
    public double getShooterCurrentRpm() { return currentRpm; }
    public double getTargetShooterRpm() { return targetShooterRpm; }
    public boolean isShooterAtSpeed() { return targetShooterRpm > 0 && Math.abs(currentRpm - targetShooterRpm) <= SHOOTER_READY_TOLERANCE_RPM; }
    public void setTargetHiveCell(Alliance alliance, HiveSide side) {
        if (activeAlliance != alliance || activeHiveSide != side) cachedTag = null;
        activeAlliance = alliance == null ? Alliance.RED : alliance; activeHiveSide = side == null ? HiveSide.FAR : side;
    }
    public Alliance toggleAlliance() { setTargetHiveCell(activeAlliance == Alliance.RED ? Alliance.BLUE : Alliance.RED, activeHiveSide); return activeAlliance; }
    public HiveSide toggleHiveSide() { setTargetHiveCell(activeAlliance, activeHiveSide == HiveSide.FAR ? HiveSide.AUDIENCE : HiveSide.FAR); return activeHiveSide; }
    public Alliance getActiveAlliance() { return activeAlliance; }
    public HiveSide getActiveHiveSide() { return activeHiveSide; }
    public static HiveSide getHiveSideForTag(Alliance alliance, int id) {
        if (alliance == Alliance.RED) {
            if (id >= 30 && id <= 33) return HiveSide.FAR;
            if (id >= 34 && id <= 37) return HiveSide.AUDIENCE;
        } else if (alliance == Alliance.BLUE) {
            if (id >= 42 && id <= 45) return HiveSide.FAR;
            if (id >= 38 && id <= 41) return HiveSide.AUDIENCE;
        }
        return null;
    }
    static LLResultTypes.FiducialResult selectAllianceFiducial(Alliance alliance, List<LLResultTypes.FiducialResult> tags) {
        if (tags == null) return null;
        for (LLResultTypes.FiducialResult tag : tags) if (tag != null && getHiveSideForTag(alliance, tag.getFiducialId()) != null) return tag;
        return null;
    }
    public static double getHiveOpeningCenterXInches(Alliance alliance) { return alliance == Alliance.BLUE ? BLUE_HIVE_PIVOT_X_IN : RED_HIVE_PIVOT_X_IN; }
    public static double getHiveOpeningCenterYInches(HiveSide side) {
        return (side == HiveSide.AUDIENCE ? -1 : 1) * HIVE_RAISED_MOUTH_CENTER_Y_OFFSET_IN;
    }
    public static double normalizeAngleDegrees(double deg) { return Math.toDegrees(RobotMath.wrap(Math.toRadians(deg))); }
    public void setLimelightPipeline(int pipeline) {
        if (limelight != null) limelight.pipelineSwitch(pipeline);
        cachedResult = null; cachedTag = null; cachedObservation = null;
    }
    public void setAllServosToZeroPosition() {
        homeTurretAndZeroEncoder(); setPitchAngleDegrees(PITCH_HOME_DEG);
    }
    public void stop() { homing.invalidate(); control.reset(); RobotShutdown.stopAll(this::stopShooter, limelight == null ? null : limelight::stop); }
    /** Compact match view: homing, flywheel readiness, launch elevation and battery. */
    public void addDriverTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret", isHomed() ? "HOMED | heading %.1f deg" : "%2$s", getRobotHeadingDegrees(), getHomingStatus());
        telemetry.addData("Flywheel (RPM)", "%.0f / %.0f | %s", currentRpm, targetShooterRpm,
                targetShooterRpm <= 0 ? "OFF" : isShooterAtSpeed() ? "READY" : "SPINNING UP");
        telemetry.addData("Launch elevation (deg)", "%.1f", hoodTravelToLaunchAngleDegrees(targetPitchDeg));
        telemetry.addData("Battery (V)", "%.1f", batteryVolts);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Turret home", getHomingStatus());
        if (visionEnabled) {
            telemetry.addData("Limelight", limelight == null ? "NOT CONFIGURED" : "ENABLED");
            telemetry.addData("Selected goal", "%s / %s | tag %d", activeAlliance, activeHiveSide, cachedTag == null ? -1 : cachedTag.getFiducialId());
            LimelightDiagnostics.addTelemetry(telemetry,cachedObservation,clock.getAsDouble());
        }
        telemetry.addData("Turret robot heading (deg CW)", "%.2f / %.2f | rate %.1f deg/s", getRobotHeadingDegrees(), getTargetRobotHeadingDegrees(), -encoderVelocity);
        telemetry.addData("Turret peak measured rate (deg/s)", "%.0f | rated %.0f no-load at 6 V", peakTurretRateDegPerSecond, TurretControl.MAX_YAW_DEG_PER_SECOND);
        telemetry.addData("Turret encoder verification", !isHomed() ? "WAITING FOR HOME"
                : isYawPositionVerified() ? "AT TARGET" : isRotationErrorHigh() ? "OUTSIDE ERROR MARGIN" : "WITHIN ERROR MARGIN");
        telemetry.addData("Turret position error (deg)", targetYawDeg - getEncoderAngleDegrees());
        telemetry.addData("Turret encoder correction", "%s | %.2f deg | correcting %s",
                control.isEncoderCorrectionEnabled() ? "ENABLED" : "DISABLED (servo position hold)",
                control.getEncoderCorrectionDegrees(), control.isEncoderCorrectionActive());
        telemetry.addData("Hood command (deg)", "%.2f | estimated tracking %s (no feedback)", targetPitchDeg, isHoodTracking());
        telemetry.addData("Launch elevation command (deg above horizontal)", hoodTravelToLaunchAngleDegrees(targetPitchDeg));
        telemetry.addData("Flywheel (RPM)", "%.0f / %.0f", currentRpm, targetShooterRpm);
        telemetry.addData("Flywheel error / ready tolerance (RPM)", "%+.0f / +/-%.0f | ready %s",
                targetShooterRpm - currentRpm, SHOOTER_READY_TOLERANCE_RPM, isShooterAtSpeed());
        telemetry.addData("Flywheel reported max (RPM at 14 V; PIDF and bang-bang)",
                ShooterSpeed.REPORTED_MAX_OBSERVED_RPM);
        telemetry.addData("Flywheel controller", shooterController);
        if (shooterController == ShooterController.BANG_BANG_FUSION) {
            telemetry.addData("Fusion zone / power", "%s | %+.0f%% | error %+.0f ticks/s (bang-bang above %.0f)",
                    targetShooterRpm == 0 ? "OFF" : fusionBangBang ? "BANG-BANG" : "FEEDFORWARD", 100 * fusionPower,
                    targetShooterRpm == 0 ? 0 : ShooterSpeed.motorTicksPerSecond(targetShooterRpm) - shooterTicksPerSecond,
                    ShooterVelocityTuning.FUSION_BANG_BANG_ERROR_TICKS_PER_SECOND);
            telemetry.addData("Fusion gains (power at 12 V; untuned)", "kS %.3f | kV %.6f | kP %.4f per tick/s",
                    fusionKs, fusionKv, fusionKp);
        }
        telemetry.addData("Shooter motor (RPM, encoder)", "%.0f | flywheel/motor ratio %.3f",
                currentRpm / SHOOTER_FLYWHEEL_REVS_PER_MOTOR_REV, SHOOTER_FLYWHEEL_REVS_PER_MOTOR_REV);
        telemetry.addData(shooterController == ShooterController.PIDF ? "Flywheel PIDF (initial gains; tune loaded)"
                        : "Flywheel PIDF (inactive in fusion mode)", "P %.1f | I %.2f | D %.1f | F %.3f | F scale %.2f",
                shooterP, shooterI, shooterD, appliedShooterF, shooterFScale);
        telemetry.addData("Shooter battery (V) / current (A)", "%.2f / %.2f (5 Hz)", batteryVolts, shooterCurrentAmps);
        telemetry.addData("Shooter limit (output RPM)", "%.0f command cap | %.0f catalog RPM",
                SHOOTER_MAX_RPM, ShooterSpeed.FLYWHEEL_MAX_RPM);
        telemetry.addData("Shooter speed headroom estimate", "%.0f%% of voltage-adjusted observed max (estimate, not measured PWM)",
                100 * ShooterVelocityTuning.estimatedFreeSpeedUsage(targetShooterRpm, batteryVolts));
        if (visionEnabled) telemetry.addData("Shot tracking", trackingStatus);
    }

    private Servo getTurretServo(HardwareMap hwMap, int expectedPort, String... candidateNames) {
        return NamedHardware.require(name -> hwMap.tryGet(Servo.class,name), () -> {
            StringBuilder configured = new StringBuilder("Expected Expansion Hub port " + expectedPort + "; inventory: ");
            for (Servo candidate : hwMap.getAll(Servo.class)) {
                configured.append(hwMap.getNamesOf(candidate)).append(" [")
                        .append(candidate.getController().getConnectionInfo()).append(", port ")
                        .append(candidate.getPortNumber()).append("]; ");
            }
            return configured.toString();
        }, candidateNames);
    }

    private <T> T tryGetDevice(HardwareMap hwMap, Class<T> clazz, String... candidateNames) {
        for (String name : candidateNames) {
            T device = hwMap.tryGet(clazz, name);
            if (device != null) {
                return device;
            }
        }
        return null;
    }
}
