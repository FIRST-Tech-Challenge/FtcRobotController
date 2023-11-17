package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;

public class HydraDrive {
    private final DcMotorEx mMotDrFrLt;
    private final DcMotorEx mMotDrFrRt;
    private final DcMotorEx mMotDrBkLt;
    private final DcMotorEx mMotDrBkRt;
    private final double mBoostedPower;
    private final double mNormalPower;
    private final double mSlowPower;
    private final double mCountsPerInch;
    private HydraOpMode mOp;

    public HydraDrive(HydraOpMode op, String frontLeft, String frontRight, String backLeft, String backRight,
                     double countsPerInch, double driveBoosted, double driveNormal, double driveSlow) {
        mOp = op;
        // grab the motors out of the hardware map
        mMotDrFrLt = (DcMotorEx)mOp.mHardwareMap.get(DcMotor.class, frontLeft);
        mMotDrFrRt = (DcMotorEx)mOp.mHardwareMap.get(DcMotor.class, frontRight);
        mMotDrBkLt = (DcMotorEx)mOp.mHardwareMap.get(DcMotor.class, backLeft);
        mMotDrBkRt = (DcMotorEx)mOp.mHardwareMap.get(DcMotor.class, backRight);
        PIDFCoefficients pid = mMotDrFrLt.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        mOp.mTelemetry.addData("FrLft PID", pid.toString());
        pid = mMotDrFrRt.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        mOp.mTelemetry.addData("FrRt PID", pid.toString());
        pid = mMotDrBkLt.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        mOp.mTelemetry.addData("BkLft PID", pid.toString());
        pid = mMotDrBkRt.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        mOp.mTelemetry.addData("BkRt PID", pid.toString());
        // store the user values for the various drive speeds
        mBoostedPower = driveBoosted;
        mNormalPower = driveNormal;
        mSlowPower = driveSlow;
        mCountsPerInch = countsPerInch;
        // set the motor directions
        mMotDrFrLt.setDirection(DcMotor.Direction.REVERSE);
        mMotDrBkLt.setDirection(DcMotor.Direction.REVERSE);
        mMotDrFrRt.setDirection(DcMotor.Direction.FORWARD);
        mMotDrBkRt.setDirection(DcMotor.Direction.FORWARD);
        // reset the encoders
        mMotDrFrLt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mMotDrBkLt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mMotDrFrRt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mMotDrBkRt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // we want to brake when we aren't applying power
        mMotDrFrLt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mMotDrBkLt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mMotDrFrRt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mMotDrBkRt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Start a drive action. This function does not check whether the last requested action has completed
     * Drive.Busy MUST be checked if it is necessary to do so
     * @param inDrive the distance to drive forward or backward
     * @param inStrafe the distance to strafe left or right
     * @param inRotate the amount to rotate in either direction
     */
    public void Start(int inDrive, int inStrafe, int inRotate) {
        // clean up the last drive to prepare for the next one
        mMotDrBkLt.setPower(0);
        mMotDrBkRt.setPower(0);
        mMotDrFrLt.setPower(0);
        mMotDrFrRt.setPower(0);
        mMotDrBkLt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mMotDrBkRt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mMotDrFrLt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mMotDrFrRt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Front left target position
        int frontLeftTarget = inDrive + inStrafe;
        frontLeftTarget = frontLeftTarget + inRotate;
        frontLeftTarget = (int) (frontLeftTarget * mCountsPerInch);
        mMotDrFrLt.setTargetPosition(frontLeftTarget);
        // Rear left target position
        int rearLeftTarget = inDrive - inStrafe;
        rearLeftTarget = rearLeftTarget + inRotate;
        rearLeftTarget = (int) (rearLeftTarget * mCountsPerInch);
        mMotDrBkLt.setTargetPosition(rearLeftTarget);
        // Front right target position
        int frontRightTarget = inDrive - inStrafe;
        frontRightTarget = frontRightTarget - inRotate;
        frontRightTarget = (int) (frontRightTarget * mCountsPerInch);
        mMotDrFrRt.setTargetPosition(frontRightTarget);
        // Rear right target position
        int rearRightTarget = inDrive + inStrafe;
        rearRightTarget = rearRightTarget - inRotate;
        rearRightTarget = (int) (rearRightTarget * mCountsPerInch);
        mMotDrBkRt.setTargetPosition(rearRightTarget);
        // Set power to the motors
        if (inRotate != 0) {
            mMotDrBkLt.setPower(mSlowPower);
            mMotDrBkRt.setPower(mSlowPower);
            mMotDrFrLt.setPower(mSlowPower);
            mMotDrFrRt.setPower(mSlowPower);
        } else {
            mMotDrBkLt.setPower(mNormalPower);
            mMotDrBkRt.setPower(mNormalPower);
            mMotDrFrLt.setPower(mNormalPower);
            mMotDrFrRt.setPower(mNormalPower);
        }
        // Run to position
        mMotDrBkLt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mMotDrBkRt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mMotDrFrLt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mMotDrFrRt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Are we busy driving?
     * @return true if the motors have not reached position
     */
    public boolean Busy() {
        boolean ret = false;
        // if either motor is still active, we are still busy
        if (mMotDrBkLt.isBusy() || mMotDrBkRt.isBusy() || mMotDrFrLt.isBusy() || mMotDrFrRt.isBusy()) {
            ret = true;
        }
        mOp.mTelemetry.addData("Driving", ret);
        return ret;
    }
}
