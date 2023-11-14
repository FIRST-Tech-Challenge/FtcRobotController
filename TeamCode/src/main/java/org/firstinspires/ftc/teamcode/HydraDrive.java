package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportClassToBlocks;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;

@ExportClassToBlocks
public class HydraDrive extends BlocksOpModeCompanion {
    private DcMotor mMotDrFrLt;
    private DcMotor mMotDrFrRt;
    private DcMotor mMotDrBkLt;
    private DcMotor mMotDrBkRt;
    private double mBoostedPower;
    private double mNormalPower;
    private double mSlowPower;
    private double mCountsPerInch;

    @ExportToBlocks(
            comment = "This function initializes the drive class by finding the motor objects and taking in the max drive power",
            tooltip = "Initialize the drive",
            parameterLabels = {"FLmotor", "FRmotor", "BLmotor", "BRmotor", "PwrFast", "PwrNormal", "PwrSlow"}
    )
    public void HydraDrive_Init(String frontLeft, String frontRight, String backLeft, String backRight,
                         double countsPerInch, double driveBoosted, double driveNormal, double driveSlow) {
        // grab the motors out of the hardware map
        mMotDrFrLt = hardwareMap.get(DcMotor.class, frontLeft);
        mMotDrFrRt = hardwareMap.get(DcMotor.class, frontRight);
        mMotDrBkLt = hardwareMap.get(DcMotor.class, backLeft);
        mMotDrBkRt = hardwareMap.get(DcMotor.class, backRight);
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
    @ExportToBlocks(
            comment = "This function initiates driving, strafing or rotating",
            tooltip = "Go, robot, go!",
            parameterLabels = {"Drive", "Strafe", "Rotate"}
    )
    public boolean HydraDrive_Process(int inDrive, int inStrafe, int inRotate) {
        int frontLeftTarget;
        int rearLeftTarget;
        int frontRightTarget;
        int rearRightTarget;

        // wait for the last requested drive to complete
        if (HydraDrive_IsBusy()) {
            return false;
        }
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
        frontLeftTarget = inDrive + inStrafe;
        frontLeftTarget = frontLeftTarget + inRotate;
        frontLeftTarget = (int) (frontLeftTarget * mCountsPerInch);
        mMotDrFrLt.setTargetPosition(frontLeftTarget);
        // Rear left target position
        rearLeftTarget = inDrive - inStrafe;
        rearLeftTarget = rearLeftTarget + inRotate;
        rearLeftTarget = (int) (rearLeftTarget * mCountsPerInch);
        mMotDrBkLt.setTargetPosition(rearLeftTarget);
        // Front right target position
        frontRightTarget = inDrive - inStrafe;
        frontRightTarget = frontRightTarget - inRotate;
        frontRightTarget = (int) (frontRightTarget * mCountsPerInch);
        mMotDrFrRt.setTargetPosition(frontRightTarget);
        // Rear right target position
        rearRightTarget = inDrive + inStrafe;
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
        return true;
    }
    @ExportToBlocks(
            comment = "This function returns whether a previous drive action is completed",
            tooltip = "Is the robot still driving?"
    )
    public boolean HydraDrive_IsBusy() {
        if (mMotDrBkLt.isBusy() || mMotDrBkRt.isBusy() || mMotDrFrLt.isBusy() || mMotDrFrRt.isBusy()) {
            return true;
        }
        if (false) {
            mMotDrBkLt.setPower(0);
            mMotDrBkRt.setPower(0);
            mMotDrFrLt.setPower(0);
            mMotDrFrRt.setPower(0);
            mMotDrBkLt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mMotDrBkRt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mMotDrFrLt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mMotDrFrRt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        return false;
    }
}
