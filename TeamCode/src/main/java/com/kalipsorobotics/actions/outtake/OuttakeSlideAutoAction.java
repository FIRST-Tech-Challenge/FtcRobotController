package com.kalipsorobotics.actions.outtake;

import androidx.savedstate.SavedStateRegistryOwner;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.ActionSet;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakePivotAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakeSlideAction;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.DcMotor;

public class OuttakeSlideAutoAction extends ActionSet {
    MoveLSAction moveLSAction;

    public OuttakeSlideAutoAction(OpModeUtilities opModeUtilities, Outtake outtake, boolean extend) {
        if (extend) {
            moveLSAction = new MoveLSAction(130, outtake);
        } else {
            moveLSAction = new MoveLSAction(0, outtake);
        }
    }
}