package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Outtake;

public class IntakeReadyAction extends KActionSet {

    public IntakeReadyAction(double degreeExtend, Intake intake, Outtake outtake) {
        MoveIntakeLSAction moveIntakeLSOut = new MoveIntakeLSAction(intake, degreeExtend);
        moveIntakeLSOut.setName("moveIntakeLSOut");
        this.addAction(moveIntakeLSOut);

        KServoAutoAction intakePivotDown = new KServoAutoAction(intake.getIntakePivotServo(), IntakePivotAction.INTAKE_PIVOT_DOWN_POS);
        intakePivotDown.setName("intakePivotDown");
        this.addAction(intakePivotDown);
    }
}
