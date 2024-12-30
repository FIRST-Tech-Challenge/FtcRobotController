package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;

public class SampleIntakeAction extends KActionSet {

    public SampleIntakeAction(IntakeClaw intake){
        KServoAutoAction moveSmallPivotDown = new KServoAutoAction(intake.getIntakeSmallPivotServo(), IntakeClaw.INTAKE_SMALL_PIVOT_GRAB_SAMPLE_POS);
        moveSmallPivotDown.setName("moveSmallPivotDown");
        this.addAction(moveSmallPivotDown);

        KServoAutoAction moveBigPivotDown = new KServoAutoAction(intake.getIntakeBigPivotServo(), IntakeClaw.INTAKE_BIG_PIVOT_GRAB_SAMPLE_POS);
        moveBigPivotDown.setName("moveBigPivotDown");
        moveBigPivotDown.setDependentActions(moveSmallPivotDown);
        this.addAction(moveBigPivotDown);

        WaitAction wait = new WaitAction(100);
        wait.setName("wait");
        wait.setDependentActions(moveBigPivotDown, moveSmallPivotDown);
        this.addAction(wait);

        KServoAutoAction clawClose = new KServoAutoAction(intake.getIntakeClawServo(), IntakeClaw.INTAKE_CLAW_CLOSE);
        clawClose.setName("clawClose");
        clawClose.setDependentActions(wait);
        this.addAction(clawClose);

    }
}
//big pivot moves down
//claw closes
