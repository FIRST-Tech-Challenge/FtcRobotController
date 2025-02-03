package com.kalipsorobotics.actions;

import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.outtake.MoveLSAction;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;

public class Init extends KActionSet{
    public Init(IntakeClaw intakeClaw, Outtake outtake) {

        KServoAutoAction bigSweep = new KServoAutoAction(intakeClaw.getIntakeBigSweepServo(), IntakeClaw.INTAKE_BIG_SWEEP_PARALLEL_TO_ROBOT);
        this.addAction(bigSweep);

        KServoAutoAction bigPivot = new KServoAutoAction(intakeClaw.getIntakeBigPivotServo(), IntakeClaw.INTAKE_BIG_PIVOT_RETRACT_POS);
        this.addAction(bigPivot);

        KServoAutoAction smallPivot = new KServoAutoAction(intakeClaw.getIntakeSmallPivotServo(), IntakeClaw.INTAKE_SMALL_PIVOT_RETRACT_POS);
        this.addAction(smallPivot);

        KServoAutoAction smallSweep = new KServoAutoAction(intakeClaw.getIntakeSmallSweepServo(), IntakeClaw.INTAKE_SMALL_SWEEP_RETRACT_POS);
        this.addAction(smallSweep);

        KServoAutoAction clawIntake = new KServoAutoAction(intakeClaw.getIntakeClawServo(), IntakeClaw.IntakeClawConfig.INTAKE_CLAW_OPEN);
        this.addAction(clawIntake);

        KServoAutoAction clawOuttake = new KServoAutoAction(outtake.getOuttakeClaw(), Outtake.OUTTAKE_CLAW_CLOSE);
        clawOuttake.setDependentActions(bigSweep,bigPivot,smallPivot,smallSweep,clawIntake);
        this.addAction(clawOuttake);

        KServoAutoAction hangHook1 = new KServoAutoAction(outtake.getHangHook1(), Outtake.HOOK1_DOWN_POS);
        hangHook1.setDependentActions(bigSweep,bigPivot,smallPivot,smallSweep,clawIntake);
        this.addAction(hangHook1);

        KServoAutoAction hangHook2 = new KServoAutoAction(outtake.getHangHook2(), Outtake.HOOK2_DOWN_POS);
        hangHook2.setDependentActions(bigSweep,bigPivot,smallPivot,smallSweep,clawIntake);
        this.addAction(hangHook2);

        KServoAutoAction linkage = new KServoAutoAction(intakeClaw.getIntakeLinkageServo(), IntakeClaw.INTAKE_LINKAGE_IN_POS);
        linkage.setDependentActions(clawOuttake, hangHook1, hangHook2);
        this.addAction(linkage);

        MoveLSAction moveLSAction = new MoveLSAction(outtake, Outtake.LS_DOWN_POS);
        moveLSAction.setDependentActions(clawOuttake,hangHook1,hangHook2);
        this.addAction(moveLSAction);

//        OuttakePivotAction outtakePivot = new OuttakePivotAction(outtake, Outtake.OUTTAKE_PIVOT_TRANSFER_READY_POS);
//        outtakePivot.setDependentActions(moveLSAction);
//        this.addAction(outtakePivot);

        intakeClaw.getOpModeUtilities().getTelemetry().addLine("init finished");
        intakeClaw.getOpModeUtilities().getTelemetry().update();
    }
}
