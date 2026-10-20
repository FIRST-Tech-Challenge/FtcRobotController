package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bot.BotProperties;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;

/**
 * Heading from the goBILDA Pinpoint odometry computer.
 *
 * <p>Uses the Pinpoint driver built into FTC SDK 12. Decode carried its own 1,088-line
 * copy of this driver; PORT-PLAN P4 said to try the built-in one first.
 *
 * <p>The robot must sit still for about a quarter second after INIT while the Pinpoint
 * calibrates its gyro. {@link #Init()} returns false until it is ready.
 */
public class Imu_Pinpoint implements Imu {
    private final HydraOpMode mOp;
    private final GoBildaPinpointDriver mPinpoint;
    private boolean mReady;
    /** Raw Pinpoint heading at the last reset. That direction counts as 0. */
    private double mHeadingAtReset;
    /** Heading read this loop, so every reader in the loop sees the same number. */
    private double mYawDegrees;

    public Imu_Pinpoint(HydraOpMode op, BotProperties properties) {
        mOp = op;
        mPinpoint = op.mHardwareMap.get(GoBildaPinpointDriver.class, properties.cfgPinpoint);
        Configure(mPinpoint, properties);

        mReady = false;
        mHeadingAtReset = 0;
        mYawDegrees = 0;
    }

    /**
     * Tell the Pinpoint where its pods are and how many ticks make an inch, then zero it.
     * Same setup as Decode's PinpointLocalizer constructor. {@code Test: Pinpoint} uses this
     * too, so the test sees exactly what the drive sees.
     */
    public static void Configure(GoBildaPinpointDriver pinpoint, BotProperties properties) {
        double mmPerTick = properties.pinpointInPerTick * 25.4;
        double forwardPodOffsetMm = mmPerTick * properties.pinpointParYTicks;
        double sidewaysPodOffsetMm = mmPerTick * properties.pinpointPerpXTicks;
        pinpoint.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);
        pinpoint.setOffsets(forwardPodOffsetMm, sidewaysPodOffsetMm, DistanceUnit.MM);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    /**
     * Wait for the Pinpoint to finish calibrating. Kept from Decode Imu_Pinpoint.InitPinpoint:
     * pick the error check for the hardware version, and on version 2 read only the
     * registers we use.
     */
    @Override
    public boolean Init() {
        if (mReady) {
            return true;
        }
        mPinpoint.update();
        GoBildaPinpointDriver.DeviceStatus status = mPinpoint.getDeviceStatus();
        switch (status) {
            case READY:
                int version = mPinpoint.getDeviceVersion();
                if (version == 2) {
                    mPinpoint.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.CRC);
                    mPinpoint.setBulkReadScope(
                            GoBildaPinpointDriver.Register.X_POSITION,
                            GoBildaPinpointDriver.Register.Y_POSITION,
                            GoBildaPinpointDriver.Register.H_ORIENTATION,
                            GoBildaPinpointDriver.Register.X_VELOCITY,
                            GoBildaPinpointDriver.Register.Y_VELOCITY,
                            GoBildaPinpointDriver.Register.H_VELOCITY,
                            GoBildaPinpointDriver.Register.DEVICE_STATUS);
                } else {
                    mPinpoint.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.LOCAL_TEST);
                }
                mOp.mTelemetry.addData("Pinpoint version", version);
                mOp.mTelemetry.addLine("Pinpoint ready");
                mReady = true;
                break;
            case CALIBRATING:
            case NOT_READY:
                mOp.mTelemetry.addLine("Pinpoint calibrating. Keep the robot still.");
                break;
            default:
                // Any FAULT_ status: a pod is unplugged, the gyro ran away, or a read failed.
                mOp.mTelemetry.addData("Pinpoint FAULT", status);
                break;
        }
        return mReady;
    }

    @Override
    public void HandleUserInput() {
    }

    /** Read the Pinpoint once per loop. */
    @Override
    public void Process() {
        mPinpoint.update();
        double rawHeading = mPinpoint.getHeading(AngleUnit.DEGREES);
        // Keep the answer between -180 and 180.
        mYawDegrees = AngleUnit.normalizeDegrees(rawHeading - mHeadingAtReset);
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
        mHeadingAtReset = mPinpoint.getHeading(AngleUnit.DEGREES);
        mYawDegrees = 0;
    }
}
