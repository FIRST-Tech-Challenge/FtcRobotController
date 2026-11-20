package org.firstinspires.ftc.teamcode;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Autonomous drivetrain subsystem. Only readSensors() performs the single Pinpoint I2C read. */
public final class PedroDrive {
    public static final double MAX_POSE_AGE_SECONDS = .1;
    private final MecanumDrivetrain motors;
    private final PinpointOdometry odometry;
    private final PedroMecanumAdapter drivetrain;
    private final ForesightConfig config = PedroConfiguration.create();
    private final Follower follower;
    private Pose pendingStart;
    private boolean referencePending;
    private boolean localReference, poseInitialized;

    public PedroDrive(HardwareMap map) {
        this(map, true);
    }
    public PedroDrive(HardwareMap map, boolean absoluteStartReference) {
        this(map, absoluteStartReference, PedroConfiguration.MAX_WHEEL_POWER, false);
    }
    /**
     * Path-planner autos. maxWheelPower: per-wheel power cap (0..1]. Paths are also limited to the speed that cap
     * allows (maxWheelPower x the tuned top speed): with speed 1.0 under a 0.35 cap, Foresight planned a ~70 in/s
     * profile the capped wheels could not follow, and the robot overshot sideways without settling (2026-11-19).
     */
    public PedroDrive(HardwareMap map, boolean absoluteStartReference, double maxWheelPower) {
        this(map, absoluteStartReference, maxWheelPower, true);
    }
    private PedroDrive(HardwareMap map, boolean absoluteStartReference, double maxWheelPower, boolean limitSpeedToCap) {
        if (!(maxWheelPower > 0 && maxWheelPower <= 1)) throw new IllegalArgumentException("Wheel power cap must be in (0, 1]");
        if (limitSpeedToCap && maxWheelPower < 1)
            config.maxVelocityConstraint.set(maxWheelPower * PedroConfiguration.MAX_FORWARD_VELOCITY_IN_PER_SEC);
        localReference = !absoluteStartReference;
        motors = new MecanumDrivetrain(map);
        motors.useOpenLoopPower();
        drivetrain = new PedroMecanumAdapter(motors, maxWheelPower);
        odometry = new PinpointOdometry(map);
        follower = new Follower(new PedroPinpointAdapter(odometry::getSample, p -> {
            if (localReference) odometry.setLocalPoseInches(p.x(), p.y(), Math.toDegrees(p.heading()));
            else odometry.setPoseInches(p.x(), p.y(), Math.toDegrees(p.heading()));
        }), drivetrain, new PedroForesight(config));
    }
    public void setStartPose(Pose pose) { pendingStart = pose; }
    public void setFieldPose(Pose pose) {
        stop();
        if (odometry.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY)
            throw new IllegalStateException("setFieldPose requires a READY Pinpoint");
        localReference = false; poseInitialized = true;
        follower.setPose(pose); pendingStart = null; referencePending = true;
    }
    public boolean idle() { return "IDLE".equals(String.valueOf(follower.mode())); }
    public void readSensors() {
        odometry.update();
        referencePending = false;
        if (pendingStart != null && odometry.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
            follower.setPose(pendingStart); pendingStart = null; poseInitialized = true; referencePending = true;
        }
        follower.localizer.update(); // Cached sample only, no I2C.
    }
    public boolean ready(double now) {
        MotionSample s = sample();
        return pendingStart == null && !referencePending && s.ready && (localReference ? poseInitialized : s.fieldReferenced)
                && now >= s.time && now - s.time <= MAX_POSE_AGE_SECONDS
                && RobotMath.finite(s.x, s.y, s.heading, s.vx, s.vy, s.omega);
    }
    public void update() { follower.update(); }
    public MotionSample sample() { return odometry.getSample(); }
    public Follower follower() { return follower; }
    public Path atSpeed(Path path, double fraction) { return path.with(config.setPathSpeed(fraction)); }
    /** Foresight settings, for per-path modifiers such as {@code config().maxAccelerationConstraint.at(a)}. */
    public ForesightConfig config() { return config; }
    public void stop() { follower.stop(); drivetrain.stop(); }
    public void addTelemetry(Telemetry telemetry) {
        motors.addConfigurationTelemetry(telemetry);
        telemetry.addData("Pedro v3", "%s | tuning %s", follower.mode(), PedroConfiguration.TUNING_VERIFIED ? "VERIFIED" : "REQUIRED");
        telemetry.addData("Pedro pose (in, deg)", "%.2f, %.2f | %.1f", follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Pedro velocity (in/s, rad/s)", "%.1f, %.1f | %.2f", follower.velocity().vx, follower.velocity().vy, follower.velocity().omega);
        telemetry.addData("Pose reference / age (s)", "%d | %s | %.3f", sample().referenceId,
                sample().fieldReferenced, System.nanoTime() * 1e-9 - sample().time);
    }
}
