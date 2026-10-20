package org.firstinspires.ftc.teamcode.objects;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.types.Alliance;

/**
 * Everything a subsystem needs from the OpMode, in one object.
 *
 * <p>Each subsystem gets this in its constructor. That way a subsystem never has to
 * know which OpMode it is running in.
 */
public class HydraOpMode {
    public final Telemetry mTelemetry;
    public final HardwareMap mHardwareMap;
    public final Gamepad mDriverGamepad;
    public final Gamepad mOperatorGamepad;

    /**
     * Which alliance we are on. Read from the alliance badge switch during INIT.
     * Stays UNKNOWN until that switch is built. Nothing in the drive depends on it yet.
     */
    public Alliance mAlliance;

    /**
     * true: drive motors are limited to BotProperties.safeModeMaxPower. Set once from the Safe
     * Mode key at INIT, then never changed. Starts true so that an OpMode that never checks
     * the key is slow, not fast. See docs/decisions/0007-safe-mode-key.md.
     */
    public boolean mSafeMode;

    /** How long the last loop took, in milliseconds. */
    public double mLoopTime;

    public HydraOpMode(Telemetry telemetry, HardwareMap hardwareMap,
                       Gamepad driverGamepad, Gamepad operatorGamepad) {
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;
        mDriverGamepad = driverGamepad;
        mOperatorGamepad = operatorGamepad;
        mAlliance = Alliance.UNKNOWN;
        mSafeMode = true;
        mLoopTime = 0;
    }
}
