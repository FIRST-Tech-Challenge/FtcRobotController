package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.bot.BotProperties;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.Subsystem;

/**
 * Checks the Safe Mode key ONE time, when INIT is pressed, and sets
 * {@code HydraOpMode.mSafeMode}.
 *
 * <p>Think "Remove Before Flight": the key is a safety pin.
 * <ul>
 *   <li><b>Key in</b>: Safe Mode. The drive is limited to a quarter speed.
 *   <li><b>Key removed</b>: full speed.
 * </ul>
 *
 * <p>After INIT the sensor is never read again. A bump or a loose wire in the middle of a
 * match can't change the speed. Put the key in or take it out BEFORE pressing INIT.
 * To stop the robot, use the power switch.
 *
 * <p>Hardware: a REV Magnetic Limit Switch and a 3D printed key with a magnet in it. For
 * bench testing now, a touch sensor stands in: pressing it pretends the key is removed.
 * {@code BotProperties.keyPressedMeansKeyIn} says which one is on the robot.
 * See docs/decisions/0007-safe-mode-key.md.
 *
 * <p>While {@code safeModeEnabled} in BotProperties is false, the key is not read at all:
 * full speed, and the Driver Station says SAFE MODE IS OFF (docs/decisions/0014).
 */
public class SafeModeKey implements Subsystem {
    private final HydraOpMode mOp;
    /** What the Driver Station shows. Worked out once, at INIT. */
    private final String mStatus;
    /** false while Safe Mode is switched off in BotProperties. */
    private final boolean mEnabled;

    public SafeModeKey(HydraOpMode op, BotProperties properties) {
        mOp = op;
        mEnabled = properties.safeModeEnabled;
        int limitPercent = (int) Math.round(BotProperties.safeModeMaxPower * 100);

        if (!mEnabled) {
            mOp.mSafeMode = false;
            mStatus = "SAFE MODE IS OFF (safeModeEnabled = false in BotProperties). FULL SPEED, key or no key.";
            return;
        }

        // The sensor is a local variable on purpose: we read it here, once, and never again.
        TouchSensor keySensor = op.mHardwareMap.tryGet(TouchSensor.class, BotProperties.cfgSafeModeKey);
        if (keySensor == null) {
            mOp.mSafeMode = true;
            mStatus = "SAFE MODE: no REV Touch Sensor named '" + BotProperties.cfgSafeModeKey
                    + "' in the robot configuration. Drive limited to " + limitPercent + "%.";
            return;
        }

        boolean keyIsIn = KeyIsIn(keySensor.isPressed(), BotProperties.keyPressedMeansKeyIn);
        if (keyIsIn) {
            mOp.mSafeMode = true;
            mStatus = "SAFE MODE: key is in. Drive limited to " + limitPercent + "%.";
        } else {
            mOp.mSafeMode = false;
            mStatus = "FULL SPEED: Safe Mode key is removed.";
        }
    }

    /**
     * Work out whether the key is in, from the sensor reading.
     *
     * @param sensorPressed what the sensor says right now
     * @param pressedMeansKeyIn true for the magnetic switch, false for the touch stand-in
     */
    static boolean KeyIsIn(boolean sensorPressed, boolean pressedMeansKeyIn) {
        if (pressedMeansKeyIn) {
            return sensorPressed;
        }
        return !sensorPressed;
    }

    /** Put the Safe Mode line on the Driver Station. Call every loop while waiting for START. */
    public void ShowStatus() {
        mOp.mTelemetry.addLine(mStatus);
        if (!mEnabled) {
            return;
        }
        if (BotProperties.keyPressedMeansKeyIn) {
            mOp.mTelemetry.addLine("(An unplugged key sensor also reads as key removed.)");
        } else {
            mOp.mTelemetry.addLine("(Test sensor '" + BotProperties.cfgSafeModeKey
                    + "': pressed = key removed = full speed.)");
        }
    }

    @Override
    public boolean Init() {
        ShowStatus();
        return true;
    }

    @Override
    public void HandleUserInput() {
    }

    /** Nothing. The key was checked at INIT and is never read again. */
    @Override
    public void Process() {
    }
}
