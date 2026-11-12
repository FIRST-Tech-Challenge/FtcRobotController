package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.PedroConfiguration;
import org.firstinspires.ftc.teamcode.PedroForesight;
import org.firstinspires.ftc.teamcode.PedroTuningHardware;
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

/**
 * Pedro web tuner procedures for V1. With the robot connected, open http://192.168.43.1:10158
 * and run them in order: Mecanum, Pinpoint, Foresight, Tests. Each step runs as a temporary
 * "Pedro Tuning" OpMode; Driver Station STOP ends it. See doc/Autonomous.md.
 *
 * The procedures use V1's subsystems through {@link PedroTuningHardware}, never Pedro's revhub
 * Mecanum/Pinpoint classes, so results match how {@code PedroDrive} drives autonomous.
 */
public final class Tuning {
    private Tuning() {}

    /** Wheel directions, reported as MecanumDrivetrain's per-side constants. */
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    /** Pinpoint pod directions and offsets, reported as PinpointOdometry constants (mm). */
    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    /** Full-power Foresight dynamics, braking and gains for PedroConfiguration and autos/robot.json. */
    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(PedroTuningHardware::localizer, PedroTuningHardware::tunerDrivetrain);
    }

    /** The installed PedroConfiguration follower at the autonomous wheel cap. */
    @Tuner
    public static Procedure tests() {
        return new Tests(PedroTuningHardware::autonomousDrivetrain, PedroTuningHardware::localizer,
                () -> new PedroForesight(PedroConfiguration.create()));
    }
}
