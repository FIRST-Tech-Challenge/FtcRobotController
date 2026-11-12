package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * V1's drivetrain and Pinpoint for Pedro's web tuner (org.firstinspires.ftc.teamcode.pedro.Tuning).
 * Tuners measure through the same subsystems and adapters as {@link PedroDrive}: open-loop
 * {@link MecanumDrivetrain} (BRAKE at zero power), {@link PedroMecanumAdapter} mixing, and
 * {@link PinpointOdometry} with V1's measured pod offsets/directions converted by
 * {@link PedroPinpointAdapter}. So the tuned values describe how autonomous actually drives.
 */
public final class PedroTuningHardware {
    private PedroTuningHardware() {}

    /** The Foresight tuner identifies full-power dynamics; PedroDrive applies MAX_WHEEL_POWER separately. */
    public static final double TUNER_WHEEL_POWER = 1.0;

    /** Uncapped drivetrain for the Mecanum/Foresight tuners. */
    public static Drivetrain tunerDrivetrain(HardwareMap map) {
        return drivetrain(map, TUNER_WHEEL_POWER);
    }

    /** Autonomous wheel cap, for verifying the tuned follower as autos will run it. */
    public static Drivetrain autonomousDrivetrain(HardwareMap map) {
        return drivetrain(map, PedroConfiguration.MAX_WHEEL_POWER);
    }

    static PedroMecanumAdapter drivetrain(HardwareMap map, double wheelPowerLimit) {
        MecanumDrivetrain motors = new MecanumDrivetrain(map);
        motors.useOpenLoopPower();
        return new PedroMecanumAdapter(motors, wheelPowerLimit);
    }

    /**
     * Pinpoint localizer whose update() performs the single Pinpoint I2C read, then converts it.
     * Tuners and Follower.update() call update() once per loop. Construction recalibrates the IMU
     * (about 0.25 s); every tuner waits 1 s before starting and re-seeds the pose after start.
     */
    public static Localizer localizer(HardwareMap map) {
        PinpointOdometry odometry = new PinpointOdometry(map);
        PedroPinpointAdapter converted = new PedroPinpointAdapter(odometry::getSample,
                p -> odometry.setLocalPoseInches(p.x(), p.y(), Math.toDegrees(p.heading())));
        return new Localizer() {
            @Override public void update() { odometry.update(); converted.update(); }
            @Override public MotionState state() { return converted.state(); }
            @Override public void setPose(Pose pose) { converted.setPose(pose); }
            @Override public void reset() { converted.reset(); }
        };
    }
}
