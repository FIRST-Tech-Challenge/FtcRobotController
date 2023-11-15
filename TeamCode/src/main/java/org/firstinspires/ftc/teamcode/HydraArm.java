package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import com.qualcomm.robotcore.hardware.DcMotor;

public class HydraArm extends BlocksOpModeCompanion {
    private final DcMotor mMotLwrArm;
    private final DcMotor mMotUprArm;
    private final double mLowerArmAutoMotorPwr;
    private final double mUpperArmAutoMotorPwr;
    HydraArmPositions mArmPositionState;
    private final int cLowerArmPos0Home = 0;
    private final int cUpperArmPos0Home = 0;
    //
    private final int cLowerArmPos1LiftBox = 0;
    private final int cUpperArmPos1LiftBox = -120;
    //
    private final int cLowerArmPos2LiftArm = -450;
    private final int cUpperArmPos2LiftArm = -120;
    //
    private final int cLowerArmPos3BackScore = -1500;
    // this is different in drive?
    private final int cUpperArmPos3BackScore = -440;
    //
    private final int cLowerArmPos4FrontScore = -700;
    private final int cUpperArmPos4FrontScore = 750;
    //
    private final int cLowerArmPos5Hang = -1200;
    private final int cUpperArmPos5Hang = 850;
    //
    private final int cLowerArmPos6Hang = -325;
    private final int cUpperArmPos6Hang = 225;
    private final int[] mLowerArmPositions = {cLowerArmPos0Home, cLowerArmPos1LiftBox, cLowerArmPos2LiftArm,
            cLowerArmPos3BackScore, cLowerArmPos4FrontScore, cLowerArmPos5Hang, cLowerArmPos6Hang};
    private final int[] mUpperArmPositions= {cUpperArmPos0Home, cUpperArmPos1LiftBox, cUpperArmPos2LiftArm,
            cUpperArmPos3BackScore, cUpperArmPos4FrontScore, cUpperArmPos5Hang, cUpperArmPos6Hang};
    private final String[] mArmPositionNames = {"Home", "Lift Box", "Lift Arm", "Back Score", "Front Score",
            "Hang", "Hang End"};

    public HydraArm(String uprArm, String lwrArm, double uprArmPwr, double lwrArmPwr) {
        mMotUprArm = hardwareMap.get(DcMotor.class, uprArm);
        mMotLwrArm = hardwareMap.get(DcMotor.class, lwrArm);
        mLowerArmAutoMotorPwr = lwrArmPwr;
        mUpperArmAutoMotorPwr = uprArmPwr;
        mArmPositionState = HydraArmPositions.ArmPosition0Home;
        mMotLwrArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mMotUprArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mMotLwrArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mMotUprArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Describe this function...
     */
    private void SetLwrArmPos(int inLwrArmPos) {
        mMotLwrArm.setTargetPosition(inLwrArmPos);
        mMotLwrArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mMotLwrArm.setPower(mLowerArmAutoMotorPwr);
    }

    /**
     * Describe this function...
     */
    private void SetUprArmPos(int inUprArmPos) {
        mMotUprArm.setTargetPosition(inUprArmPos);
        mMotUprArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mMotUprArm.setPower(mUpperArmAutoMotorPwr);
    }

    public boolean RunAction(HydraArmMovements action) {
        if (Busy()) {
            return false;
        }
        boolean actionComplete = false;
        switch (action) {
            case ArmMoveToHome:
                switch (mArmPositionState) {
                    case ArmPosition0Home:
                        actionComplete = true;
                        break;
                    case ArmPosition1LiftBox:
                        mArmPositionState = HydraArmPositions.ArmPosition0Home;
                        break;
                    case ArmPosition2LiftArm:
                        mArmPositionState = HydraArmPositions.ArmPosition1LiftBox;
                        break;
                    case ArmPosition3BackScore:
                    case ArmPosition4FrontScore:
                    case ArmPosition5Hang:
                        mArmPositionState = HydraArmPositions.ArmPosition2LiftArm;
                        break;
                    case ArmPosition6HangEnd:
                        mArmPositionState = HydraArmPositions.ArmPosition5Hang;
                        break;
                }
                break;
            case ArmMoveToBack:
                switch (mArmPositionState) {
                    case ArmPosition0Home:
                        mArmPositionState = HydraArmPositions.ArmPosition1LiftBox;
                        break;
                    case ArmPosition1LiftBox:
                        mArmPositionState = HydraArmPositions.ArmPosition2LiftArm;
                        break;
                    case ArmPosition2LiftArm:
                    case ArmPosition4FrontScore:
                    case ArmPosition5Hang:
                        mArmPositionState = HydraArmPositions.ArmPosition3BackScore;
                        break;
                    case ArmPosition3BackScore:
                        actionComplete = true;
                        break;
                    case ArmPosition6HangEnd:
                        mArmPositionState = HydraArmPositions.ArmPosition5Hang;
                        break;
                }
                break;
            case ArmMoveToFront:
                switch (mArmPositionState) {
                    case ArmPosition0Home:
                        mArmPositionState = HydraArmPositions.ArmPosition1LiftBox;
                        break;
                    case ArmPosition1LiftBox:
                        mArmPositionState = HydraArmPositions.ArmPosition2LiftArm;
                        break;
                    case ArmPosition2LiftArm:
                    case ArmPosition3BackScore:
                    case ArmPosition5Hang:
                        mArmPositionState = HydraArmPositions.ArmPosition4FrontScore;
                        break;
                    case ArmPosition4FrontScore:
                        actionComplete = true;
                        break;
                    case ArmPosition6HangEnd:
                        mArmPositionState = HydraArmPositions.ArmPosition5Hang;
                        break;
                }
                break;
            case ArmMoveToHang:
                switch (mArmPositionState) {
                    case ArmPosition0Home:
                        mArmPositionState = HydraArmPositions.ArmPosition1LiftBox;
                        break;
                    case ArmPosition1LiftBox:
                        mArmPositionState = HydraArmPositions.ArmPosition2LiftArm;
                        break;
                    case ArmPosition2LiftArm:
                    case ArmPosition3BackScore:
                    case ArmPosition4FrontScore:
                    case ArmPosition6HangEnd:
                        mArmPositionState = HydraArmPositions.ArmPosition5Hang;
                        break;
                    case ArmPosition5Hang:
                        actionComplete = true;
                        break;
                }
                break;
            case ArmMoveToHangEnd:
                switch (mArmPositionState) {
                    case ArmPosition0Home:
                        mArmPositionState = HydraArmPositions.ArmPosition1LiftBox;
                        break;
                    case ArmPosition1LiftBox:
                        mArmPositionState = HydraArmPositions.ArmPosition2LiftArm;
                        break;
                    case ArmPosition2LiftArm:
                    case ArmPosition3BackScore:
                    case ArmPosition4FrontScore:
                        mArmPositionState = HydraArmPositions.ArmPosition5Hang;
                        break;
                    case ArmPosition5Hang:
                        mArmPositionState = HydraArmPositions.ArmPosition6HangEnd;
                        break;
                    case ArmPosition6HangEnd:
                        actionComplete = true;
                        break;
                }
                break;
        }
        SetLwrArmPos(mLowerArmPositions[mArmPositionState.ordinal()]);
        SetUprArmPos(mUpperArmPositions[mArmPositionState.ordinal()]);
        telemetry.addData("ArmPos", mArmPositionNames[mArmPositionState.ordinal()]);
        telemetry.addData("LrArm", mMotLwrArm.getCurrentPosition());
        telemetry.addData("UprArm", mMotUprArm.getCurrentPosition());
        return actionComplete;
    }

    /**
     * Describe this function...
     */
    private boolean Busy() {
        int upperArmError = Math.abs(mMotUprArm.getTargetPosition() - mMotUprArm.getCurrentPosition());
        int lowerArmError = Math.abs(mMotLwrArm.getTargetPosition() - mMotLwrArm.getCurrentPosition());
        if (lowerArmError <= 15 && upperArmError <= 15) {
            return false;
        }
        if (mMotLwrArm.isBusy() || mMotUprArm.isBusy()) {
            return true;
        }
        return false;
    }
}
