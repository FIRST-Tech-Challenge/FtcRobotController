package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
    private final HydraOpMode mOp;
    protected final double cRampDownStartPercentage = 0.9;
    protected final double cRampLowPower = 0.3;
    protected final double cRampUpRate = 0.05;
    protected final double cRampDownRate = 0.05;
    protected int mRampDownStart;
    protected double mCurrentDrivePower;
    protected double mCurrentDriveMaxPower;

    public HydraDrive(HydraOpMode op, String frontLeft, String frontRight, String backLeft, String backRight,
                     double countsPerInch, double driveBoosted, double driveNormal, double driveSlow) {
        mOp = op;
        // grab the motors out of the hardware map
        mMotDrFrLt = (DcMotorEx)mOp.mHardwareMap.get(DcMotor.class, frontLeft);
        mMotDrFrRt = (DcMotorEx)mOp.mHardwareMap.get(DcMotor.class, frontRight);
        mMotDrBkLt = (DcMotorEx)mOp.mHardwareMap.get(DcMotor.class, backLeft);
        mMotDrBkRt = (DcMotorEx)mOp.mHardwareMap.get(DcMotor.class, backRight);
        // Grab the PID coefficients so we can play with them
        PIDFCoefficients pid = mMotDrFrLt.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        mOp.mTelemetry.addData("FrLft PID", pid.toString());
        pid = mMotDrFrRt.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        mOp.mTelemetry.addData("FrRt PID", pid.toString());
        pid = mMotDrBkLt.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        mOp.mTelemetry.addData("BkLft PID", pid.toString());
        pid = mMotDrBkRt.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        mOp.mTelemetry.addData("BkRt PID", pid.toString());
        if (false) {
            // todo try setting new PID values
            PIDFCoefficients newPIDF = new PIDFCoefficients(pid.p, pid.i, pid.d, pid.f, MotorControlAlgorithm.PIDF);
            mMotDrBkLt.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, newPIDF);
            mMotDrBkRt.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, newPIDF);
            mMotDrFrLt.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, newPIDF);
            mMotDrFrRt.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, newPIDF);
        }
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
        SetAllMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        // Clean up the last drive to prepare for the next one
        SetAllMotorPower(0);
        SetAllMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Front left target position
        int frontLeftTarget = (int)((inDrive + inStrafe + inRotate) * mCountsPerInch);
        mMotDrFrLt.setTargetPosition(frontLeftTarget);
        // Rear left target position
        int rearLeftTarget = (int)((inDrive - inStrafe + inRotate) * mCountsPerInch);
        mMotDrBkLt.setTargetPosition(rearLeftTarget);
        // Front right target position
        int frontRightTarget = (int)((inDrive - inStrafe - inRotate) * mCountsPerInch);
        mMotDrFrRt.setTargetPosition(frontRightTarget);
        // Rear right target position
        int rearRightTarget = (int)((inDrive + inStrafe - inRotate) * mCountsPerInch);
        mMotDrBkRt.setTargetPosition(rearRightTarget);
        // Get the total drive so we can calculate when to ramp the power down
        int totalDrive = Math.abs(frontLeftTarget) + Math.abs(rearLeftTarget) + Math.abs(frontRightTarget) + Math.abs(rearRightTarget);
        // Start ramping down when the error is under this value
        mRampDownStart = (int)((1 - cRampDownStartPercentage) * totalDrive);
        // Start at this power
        mCurrentDrivePower = cRampLowPower;
        // Ramp up to this power
        if (inRotate != 0) {
            mCurrentDriveMaxPower = mSlowPower;
        } else {
            mCurrentDriveMaxPower = mNormalPower;
        }
        // Set power
        SetAllMotorPower(mCurrentDrivePower);
        // Run to position
        SetAllMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Are we busy driving?
     * @return true if the motors have not reached position
     */
    public boolean Busy() {
        boolean ret = false;
        // Get the target position for each motor
        int FLmotTarget = mMotDrFrLt.getTargetPosition();
        int FRmotTarget = mMotDrFrRt.getTargetPosition();
        int BLmotTarget = mMotDrBkLt.getTargetPosition();
        int BRmotTarget = mMotDrBkRt.getTargetPosition();
        // Get the current position of each motor
        int FLmotPos = mMotDrFrLt.getCurrentPosition();
        int FRmotPos = mMotDrFrRt.getCurrentPosition();
        int BLmotPos = mMotDrBkLt.getCurrentPosition();
        int BRmotPos = mMotDrBkRt.getCurrentPosition();
        /*mOp.mTelemetry.addData("FLmotTarget",FLmotTarget);
        mOp.mTelemetry.addData("FRmotTarget",FRmotTarget);
        mOp.mTelemetry.addData("BLmotTarget",BLmotTarget);
        mOp.mTelemetry.addData("BRmotTarget",BRmotTarget);
        mOp.mTelemetry.addData("FLmotPos",FLmotPos);
        mOp.mTelemetry.addData("FRmotPos",FRmotPos);
        mOp.mTelemetry.addData("BLmotPos",BLmotPos);
        mOp.mTelemetry.addData("BRmotPos",BRmotPos);*/
        // Calculate the current error for each motor
        int errorBkLt = Math.abs(BLmotTarget) - Math.abs(BLmotPos);
        int errorBkRt = Math.abs(BRmotTarget) - Math.abs(BRmotPos);
        int errorFrLt = Math.abs(FLmotTarget) - Math.abs(FLmotPos);
        int errorFrRt = Math.abs(FRmotTarget) - Math.abs(FRmotPos);
        // Total error for all motors
        int totalError = errorBkLt + errorBkRt + errorFrLt + errorFrRt;
        // Ramp
        if (totalError < mRampDownStart) {
            // Ramp down at the end
            if (mCurrentDrivePower > cRampLowPower) {
                mCurrentDrivePower -= cRampDownRate;
                if (mCurrentDrivePower < cRampLowPower) {
                    mCurrentDrivePower = cRampLowPower;
                }
                SetAllMotorPower(mCurrentDrivePower);
            }
        }
        else if (mCurrentDrivePower < mCurrentDriveMaxPower) {
            // Ramp up at the beginning
            mCurrentDrivePower += cRampUpRate;
            if (mCurrentDrivePower > mCurrentDriveMaxPower) {
                mCurrentDrivePower = mCurrentDriveMaxPower;
            }
            SetAllMotorPower(mCurrentDrivePower);

        }
        // if any motor is still active, we are still busy
        if (mMotDrBkLt.isBusy() || mMotDrBkRt.isBusy() || mMotDrFrLt.isBusy() || mMotDrFrRt.isBusy()) {
            ret = true;
        }
        mOp.mTelemetry.addData("Driving", ret);
        return ret;
    }

    /**
     * Sets the power to all motors
     * @param value the power to set to the motors
     */
    private void SetAllMotorPower(double value) {
        mMotDrBkLt.setPower(value);
        mMotDrBkRt.setPower(value);
        mMotDrFrLt.setPower(value);
        mMotDrFrRt.setPower(value);
    }

    /**
     * Sets the run mode for all motors
     * @param value the run mode to set
     */
    private void SetAllMotorMode(DcMotor.RunMode value) {
        mMotDrBkLt.setMode(value);
        mMotDrBkRt.setMode(value);
        mMotDrFrLt.setMode(value);
        mMotDrFrRt.setMode(value);
    }
}
