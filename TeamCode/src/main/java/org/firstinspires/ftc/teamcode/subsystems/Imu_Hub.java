package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.bot.BotProperties;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;

/**
 * Heading from the IMU built into the REV Control Hub.
 *
 * <p>Used when the robot has no Pinpoint (see {@link ImuSelector}). Ported from Decode
 * subsystems/Imu_Hub.java, which PORT-PLAN had marked to drop; it came back because the
 * test robot has no Pinpoint right now (docs/decisions/0005).
 *
 * <p>The hub must be told how it is mounted, or "turning left" can read as tilting. The
 * mounting is in BotProperties: hubLogoFacing and hubUsbFacing.
 */
public class Imu_Hub implements Imu {
    private final HydraOpMode mOp;
    private final BotProperties mProperties;
    private final IMU mImu;
    private boolean mStartAttempted;
    private boolean mReady;
    /** Heading read this loop, so every reader in the loop sees the same number. */
    private double mYawDegrees;

    public Imu_Hub(HydraOpMode op, BotProperties properties) {
        mOp = op;
        mProperties = properties;
        mImu = op.mHardwareMap.get(IMU.class, mProperties.cfgHubImu);
        mStartAttempted = false;
        mReady = false;
        mYawDegrees = 0;
    }

    /**
     * Tell the IMU how the hub is mounted and zero the heading. Tried once. If it fails,
     * INIT still finishes: the drive works, just robot-centric (heading stays 0).
     */
    @Override
    public boolean Init() {
        if (!mStartAttempted) {
            mStartAttempted = true;
            RevHubOrientationOnRobot mounting = new RevHubOrientationOnRobot(
                    mProperties.hubLogoFacing, mProperties.hubUsbFacing);
            mReady = mImu.initialize(new IMU.Parameters(mounting));
            if (mReady) {
                mImu.resetYaw();
                mYawDegrees = 0;
            }
        }

        if (mReady) {
            mOp.mTelemetry.addLine("Hub IMU ready");
        } else {
            mOp.mTelemetry.addLine("Hub IMU FAULT: did not start. Field-centric will act robot-centric.");
        }
        return true;
    }

    @Override
    public void HandleUserInput() {
    }

    /** Read the IMU once per loop. The hub IMU is not part of the bulk read, so each read costs time. */
    @Override
    public void Process() {
        if (mReady) {
            mYawDegrees = mImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }
    }

    @Override
    public boolean Ready() {
        return mReady;
    }

    @Override
    public double GetYaw() {
        return mYawDegrees;
    }

    @Override
    public void ResetYaw() {
        mImu.resetYaw();
        mYawDegrees = 0;
    }
}
