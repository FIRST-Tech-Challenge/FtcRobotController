package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.actions.outtake.teleopActions.OuttakePivotAction;
import com.kalipsorobotics.modules.Outtake;

public class BasketReadyAction {
    Outtake outtake;
    MoveLSAction moveLSAction;
    OuttakePivotAutoAction outtakePivotAutoAction;
    public BasketReadyAction() {
        moveLSAction = new MoveLSAction(outtake, 2040);
        outtakePivotAutoAction = new OuttakePivotAutoAction(outtake, 0.5);
        outtakePivotAutoAction = new OuttakePivotAutoAction(outtake, 0.05);
    }
}
