package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.bot.BotProperties;
import org.firstinspires.ftc.teamcode.objects.AimAssist;
import org.firstinspires.ftc.teamcode.objects.Debouncer;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.MecanumMath;

/**
 * Driver control of the mecanum drive.
 *
 * <p>Controls (driver gamepad):
 * <ul>
 *   <li><b>Left stick</b>: drive and strafe.
 *   <li><b>Right stick X</b>: turn.
 *   <li><b>Left trigger</b>: slow mode. <b>Right trigger</b>: boost.
 *   <li><b>Circle / B</b>: "the way the robot faces now is forward" (field-centric only).
 *   <li><b>Left bumper (hold)</b>: aim assist. The robot turns to face the nearest AprilTag
 *       the camera sees. The sticks still drive and strafe. Let go to turn by hand again.
 * </ul>
 *
 * <p>Ported from Decode subsystems/Drive_Manual.java. The math is unchanged and lives in
 * {@link MecanumMath}. Aim assist is Decode's left-bumper idea, done over: Decode's was
 * overwritten by the next line every loop. Snap-to-heading (square) is still left out;
 * see docs/decisions/0002-drive-port.md and 0011.
 */
public class Drive_Manual extends Drive {
    private final Gamepad mGamepad;
    private final Vision mVision;
    private final Debouncer mResetHeadingButton;

    // Read from the gamepad in HandleUserInput, used in Process.
    private double mDrive;
    private double mStrafe;
    private double mRotate;
    private double mSpeedLimit;
    private boolean mAimHeld;

    public Drive_Manual(HydraOpMode op, BotProperties properties, Imu imu, Vision vision) {
        super(op, properties, imu);
        mGamepad = mOp.mDriverGamepad;
        mVision = vision;
        SetAllMotorPower(0.0);
        SetAllMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mResetHeadingButton = new Debouncer(mProperties.debounce);
        mSpeedLimit = mProperties.driveNormal;
    }

    @Override
    public void HandleUserInput() {
        mDrive = mGamepad.left_stick_y;
        mStrafe = -mGamepad.left_stick_x * mProperties.driveStrafeGain;
        mRotate = -mGamepad.right_stick_x;
        mAimHeld = mGamepad.left_bumper;

        // Circle resets which way is forward. Rumble so the driver feels it happen.
        mResetHeadingButton.In(mGamepad.circle);
        if (mResetHeadingButton.Out() && mProperties.fieldCentricDrive) {
            mResetHeadingButton.Used();
            mGamepad.rumbleBlips(1);
            mImu.ResetYaw();
        }

        if (mGamepad.left_trigger > mProperties.trgBtnThresh) {
            mSpeedLimit = mProperties.driveSlow;
        } else if (mGamepad.right_trigger > mProperties.trgBtnThresh) {
            mSpeedLimit = mProperties.driveBoosted;
        } else {
            mSpeedLimit = mProperties.driveNormal;
        }
    }

    @Override
    public void Process() {
        // No heading yet? Drive as if facing forward, like Decode did.
        double yaw = 0;
        if (mImu.Ready()) {
            yaw = mImu.GetYaw();
        }

        // Left bumper: aim assist. The tag decides the turn; the sticks still drive and strafe.
        if (mAimHeld && mVision.SeesTag()) {
            mRotate = AimAssist.RotateToward(mVision.GetBearing(), mProperties.aimKP,
                    mProperties.aimMaxRotate, mProperties.aimDoneDegrees);
            mOp.mTelemetry.addData("Aim assist", "ON: tag %d, turn %.2f", mVision.GetTagId(), mRotate);
        } else if (mAimHeld) {
            mOp.mTelemetry.addLine("Aim assist: held, but no tag in view");
        }

        MecanumMath.WheelPowers powers;
        if (mProperties.fieldCentricDrive) {
            powers = MecanumMath.FieldCentric(mDrive, mStrafe, mRotate, yaw);
        } else {
            powers = MecanumMath.RobotCentric(mDrive, mStrafe, mRotate);
        }

        SetMotorPower(mFrontLeftMotor, powers.frontLeft * mSpeedLimit);
        SetMotorPower(mFrontRightMotor, powers.frontRight * mSpeedLimit);
        SetMotorPower(mBackLeftMotor, powers.backLeft * mSpeedLimit);
        SetMotorPower(mBackRightMotor, powers.backRight * mSpeedLimit);

        if (mOp.mSafeMode) {
            mOp.mTelemetry.addLine("SAFE MODE: drive limited");
        }
        mOp.mTelemetry.addData("Yaw", yaw);
        mOp.mTelemetry.addData("Power FL", mFrontLeftMotor.getPower());
        mOp.mTelemetry.addData("Power FR", mFrontRightMotor.getPower());
        mOp.mTelemetry.addData("Power BL", mBackLeftMotor.getPower());
        mOp.mTelemetry.addData("Power BR", mBackRightMotor.getPower());
    }
}
