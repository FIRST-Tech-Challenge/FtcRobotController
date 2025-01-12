package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;

public class SampleIntakeAction extends KActionSet {

    public SampleIntakeAction(IntakeClaw intake){

        KServoAutoAction intakeClawOpen = new KServoAutoAction(intake.getIntakeClawServo(), IntakeClaw.INTAKE_CLAW_OPEN);
        intakeClawOpen.setName("intakeClawOpen");
        this.addAction(intakeClawOpen);

        KServoAutoAction moveSmallPivotDown = new KServoAutoAction(intake.getIntakeSmallPivotServo(), IntakeClaw.INTAKE_SMALL_PIVOT_GRAB_SAMPLE_POS);
        moveSmallPivotDown.setName("moveSmallPivotDown");
        moveSmallPivotDown.setDependentActions(intakeClawOpen);
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

        KServoAutoAction moveBigPivot = new KServoAutoAction(intake.getIntakeBigPivotServo(), IntakeClaw.INTAKE_BIG_PIVOT_INTAKE_READY_POS - 0.07);
        moveBigPivot.setName("moveBigPivot");
        moveBigPivot.setDependentActions(clawClose);
        this.addAction(moveBigPivot);

        KServoAutoAction moveSmallPivot = new KServoAutoAction(intake.getIntakeSmallPivotServo(), IntakeClaw.INTAKE_SMALL_PIVOT_INTAKE_READY_POS);
        moveSmallPivot.setName("moveSmallPivot");
        moveSmallPivot.setDependentActions(clawClose);
        this.addAction(moveSmallPivot);

    }
}
//big pivot moves down
//claw closes
