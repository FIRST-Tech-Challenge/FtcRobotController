package com.kalipsorobotics.actions;

import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.outtake.MoveLSAction;
import com.kalipsorobotics.actions.outtake.MoveOuttakeLSAction;
import com.kalipsorobotics.math.CalculateTickPer;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;

public class Init extends KActionSet{
    public Init(IntakeClaw intakeClaw, Outtake outtake) {
        KServoAutoAction linkage = new KServoAutoAction(intakeClaw.getIntakeLinkageServo(), IntakeClaw.INTAKE_LINKAGE_IN_POS);
        this.addAction(linkage);

        KServoAutoAction bigSweep = new KServoAutoAction(intakeClaw.getIntakeBigSweepServo(), IntakeClaw.INTAKE_BIG_SWEEP_PARALLEL_TO_ROBOT);
        this.addAction(bigSweep);

        KServoAutoAction bigPivot = new KServoAutoAction(intakeClaw.getIntakeBigPivotServo(), IntakeClaw.INTAKE_BIG_PIVOT_RETRACT_POS);
        this.addAction(bigPivot);

        KServoAutoAction smallPivot = new KServoAutoAction(intakeClaw.getIntakeSmallPivotServo(), IntakeClaw.INTAKE_SMALL_PIVOT_RETRACT_POS);
        this.addAction(smallPivot);

        KServoAutoAction smallSweep = new KServoAutoAction(intakeClaw.getIntakeSmallSweepServo(), IntakeClaw.INTAKE_SMALL_SWEEP_RETRACT_POS);
        this.addAction(smallSweep);

        KServoAutoAction clawIntake = new KServoAutoAction(intakeClaw.getIntakeClawServo(), IntakeClaw.INTAKE_CLAW_OPEN);
        this.addAction(clawIntake);

        KServoAutoAction clawOuttake = new KServoAutoAction(outtake.getOuttakeClaw(), Outtake.OUTTAKE_CLAW_CLOSE);
        clawOuttake.setDependentActions(linkage,bigSweep,bigPivot,smallPivot,smallSweep,clawIntake);
        this.addAction(clawOuttake);

        KServoAutoAction outtakePivot = new KServoAutoAction(outtake.getOuttakePivotServo(), Outtake.OUTTAKE_PIVOT_TRANSFER_READY_POS);
        outtakePivot.setDependentActions(linkage,bigSweep,bigPivot,smallPivot,smallSweep,clawIntake);
        this.addAction(outtakePivot);

        KServoAutoAction hangHook1 = new KServoAutoAction(outtake.getHangHook1(), Outtake.HOOK1_DOWN_POS);
        hangHook1.setDependentActions(linkage,bigSweep,bigPivot,smallPivot,smallSweep,clawIntake);
        this.addAction(hangHook1);

        KServoAutoAction hangHook2 = new KServoAutoAction(outtake.getHangHook2(), Outtake.HOOK2_DOWN_POS);
        hangHook2.setDependentActions(linkage,bigSweep,bigPivot,smallPivot,smallSweep,clawIntake);
        this.addAction(hangHook2);

        MoveLSAction moveLSAction = new MoveLSAction(outtake, Outtake.LS_DOWN_POS);
        moveLSAction.setDependentActions(clawOuttake,outtakePivot,hangHook1,hangHook2);
        this.addAction(moveLSAction);
    }
}
