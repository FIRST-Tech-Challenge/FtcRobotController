package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.ActionSet;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.OpModeUtilities;

public class SpecimenHangAction extends ActionSet {
    OpModeUtilities opModeUtilities;
    Outtake outtake;
    OuttakeSlideAutoAction outtakeSlideAutoActionGoUp;
    OuttakeSlideAutoAction outtakeSlideAutoActionGoDown;
    OuttakePivotAutoAction outtakePivotAutoAction;

    public SpecimenHangAction(OpModeUtilities opModeUtilities) {
        this.opModeUtilities = opModeUtilities;
        this.outtake = new Outtake(this.opModeUtilities);
        this.outtakeSlideAutoActionGoUp = new OuttakeSlideAutoAction(this.outtake);
        this.outtakeSlideAutoActionGoDown = new OuttakeSlideAutoAction(this.outtake);
        outtakePivotAutoAction = new OuttakePivotAutoAction(outtake, OuttakePivotAutoAction.Position.SPECIMEN);
        this.scheduleSequential(this.outtakeSlideAutoActionGoUp);
        this.scheduleSequential(this.outtakePivotAutoAction);
        this.scheduleSequential(this.outtakeSlideAutoActionGoDown);
    }

    public SpecimenHangAction() {

    }
    //TODO somebody do this --place holder for now
    @Override
    public boolean checkDoneCondition() {
        return true;
    }
}
