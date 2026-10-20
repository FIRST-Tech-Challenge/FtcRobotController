package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.bot.BotProperties;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;

/**
 * The Vision for a robot with no camera. It never sees a tag, so aim assist does nothing,
 * and the drive still works. The Driver Station says why during INIT.
 */
public class Vision_None implements Vision {
    private final HydraOpMode mOp;
    private final BotProperties mProperties;

    public Vision_None(HydraOpMode op, BotProperties properties) {
        mOp = op;
        mProperties = properties;
    }

    @Override
    public boolean Init() {
        mOp.mTelemetry.addLine("No camera: nothing named '" + mProperties.cfgLimelight + "' or '"
                + mProperties.cfgWebcam + "' in the robot configuration. Aim assist is off.");
        return true;
    }

    @Override
    public void HandleUserInput() {
    }

    @Override
    public void Process() {
    }

    @Override
    public boolean SeesTag() {
        return false;
    }

    @Override
    public int GetTagId() {
        return 0;
    }

    @Override
    public double GetBearing() {
        return 0;
    }

    @Override
    public double GetRange() {
        return 0;
    }

    @Override
    public int CountTags() {
        return 0;
    }
}
