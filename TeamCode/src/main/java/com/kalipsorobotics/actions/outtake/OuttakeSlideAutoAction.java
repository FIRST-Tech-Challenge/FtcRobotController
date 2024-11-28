package com.kalipsorobotics.actions.outtake;

import androidx.savedstate.SavedStateRegistryOwner;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakePivotAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakeSlideAction;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.DcMotor;

public class OuttakeSlideAutoAction extends Action {
    OpModeUtilities opModeUtilities;
    OuttakeSlideAction outtakeSlideAction;
    DcMotor linearSlideMotor1;
    DcMotor linearSlideMotor2;

    @Override
    public boolean checkDoneCondition() {
        return false;
    }
    public OuttakeSlideAutoAction(OpModeUtilities opModeUtilities, Outtake outtake) {
        this.opModeUtilities = opModeUtilities;
        this.outtakeSlideAction = new OuttakeSlideAction(outtake);
        this.linearSlideMotor1 = outtake.getLinearSlideMotor1();
        this.linearSlideMotor2 = outtake.getLinearSlide2();
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setMotorMode(DcMotor.RunMode mode) {
        linearSlideMotor1.setMode(mode);
        linearSlideMotor2.setMode(mode);
    }
    public void setTargetPosition(int position) {
        linearSlideMotor1.setTargetPosition(position);
        linearSlideMotor2.setTargetPosition(position);
    }
    public void MoveToPosition(int position) {
        setTargetPosition(position);
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
