package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.bot.BotProperties;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.MotorSlewRate;
import org.firstinspires.ftc.teamcode.objects.Subsystem;

/**
 * The four mecanum drive motors. Sets them up; does not decide how to move.
 * {@link Drive_Manual} adds driver control on top.
 *
 * <p>Every drive power goes through {@link #SetMotorPower} or {@link #SetAllMotorPower},
 * so Safe Mode applies everywhere: TeleOp and Auto (docs/decisions/0004).
 *
 * <p>Ported from Decode subsystems/Drive.java. Left out on purpose: the {@code if (false)}
 * PID block (PORT-PLAN P6 cut list), the encoder ramp fields nothing read, and the
 * RUN_TO_POSITION tolerance (the drive never runs in that mode).
 */
public class Drive implements Subsystem {
    protected final HydraOpMode mOp;
    protected final BotProperties mProperties;
    protected final Imu mImu;
    protected final DcMotorEx mFrontLeftMotor;
    protected final DcMotorEx mFrontRightMotor;
    protected final DcMotorEx mBackLeftMotor;
    protected final DcMotorEx mBackRightMotor;

    public Drive(HydraOpMode op, BotProperties properties, Imu imu) {
        mOp = op;
        mProperties = properties;
        mImu = imu;
        mFrontLeftMotor = mOp.mHardwareMap.get(DcMotorEx.class, mProperties.cfgDriveFrontLeft);
        mFrontRightMotor = mOp.mHardwareMap.get(DcMotorEx.class, mProperties.cfgDriveFrontRight);
        mBackLeftMotor = mOp.mHardwareMap.get(DcMotorEx.class, mProperties.cfgDriveBackLeft);
        mBackRightMotor = mOp.mHardwareMap.get(DcMotorEx.class, mProperties.cfgDriveBackRight);

        // Left side forward, right side reverse. MecanumMath's signs depend on this.
        mFrontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        mBackLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        mFrontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        mBackRightMotor.setDirection(DcMotor.Direction.REVERSE);

        SetAllMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Hold still instead of coasting when the stick is released.
        mFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public boolean Init() {
        return true;
    }

    @Override
    public void HandleUserInput() {
    }

    @Override
    public void Process() {
    }

    /**
     * The Safe Mode speed limit. Every drive power passes through here.
     *
     * <p>In Safe Mode the power is multiplied by a quarter, so the stick still feels smooth
     * from 0 to full. Then it is clamped, so nothing can get past the limit, not even
     * field-centric's 1.41. Outside Safe Mode the power passes through unchanged.
     *
     * @return the power to send to the motor. A broken number (NaN) becomes 0, stopped.
     */
    public static double LimitPower(double power, boolean safeMode) {
        if (Double.isNaN(power)) {
            return 0;
        }
        if (!safeMode) {
            return power;
        }
        double limit = BotProperties.safeModeMaxPower;
        double scaled = power * limit;
        return Math.max(-limit, Math.min(scaled, limit));
    }

    /** Set the same power on all four motors. 0 stops the robot. Safe Mode applies. */
    protected void SetAllMotorPower(double power) {
        double safePower = LimitPower(power, mOp.mSafeMode);
        mFrontLeftMotor.setPower(safePower);
        mFrontRightMotor.setPower(safePower);
        mBackLeftMotor.setPower(safePower);
        mBackRightMotor.setPower(safePower);
    }

    /** Move one drive motor toward a power, a little each loop. Safe Mode applies. */
    protected void SetMotorPower(DcMotorEx motor, double power) {
        double safePower = LimitPower(power, mOp.mSafeMode);
        MotorSlewRate.Set(motor, safePower, mProperties.driveMotorSlewRate);
    }

    /** Set the run mode on all four motors. */
    protected void SetAllMotorMode(DcMotor.RunMode mode) {
        mFrontLeftMotor.setMode(mode);
        mFrontRightMotor.setMode(mode);
        mBackLeftMotor.setMode(mode);
        mBackRightMotor.setMode(mode);
    }
}
