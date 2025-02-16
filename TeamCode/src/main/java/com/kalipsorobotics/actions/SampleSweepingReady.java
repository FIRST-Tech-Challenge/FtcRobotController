package com.kalipsorobotics.actions;

import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.modules.IntakeClaw;

public class SampleSweepingReady extends KActionSet{

    public SampleSweepingReady(IntakeClaw intake, double intakeSmallSweepServoPos, double intakeClawPos){

        KServoAutoAction pushRatchet = new KServoAutoAction(intake.getIntakeRatchetServo(), IntakeClaw.INTAKE_RATCHET_UNLOCK_POS);
        pushRatchet.setName("pushRatchet");
        this.addAction(pushRatchet);

        KServoAutoAction moveOpenClaw = new KServoAutoAction(intake.getIntakeClawServo(), intakeClawPos);
        moveOpenClaw.setName("moveOpenClaw");
        this.addAction(moveOpenClaw);

        KServoAutoAction moveBigPivot1 = new KServoAutoAction(intake.getIntakeBigPivotServo(), IntakeClaw.INTAKE_BIG_PIVOT_RETRACT_POS);
        moveBigPivot1.setName("moveBigPivot1");
        this.addAction(moveBigPivot1);
        //moveBigPivot1.setDependentActions(smallPivotHeadStart);

        KServoAutoAction moveSmallPivot1 = new KServoAutoAction(intake.getIntakeSmallPivotServo(), IntakeClaw.INTAKE_SMALL_PIVOT_RETRACT_POS);
        moveSmallPivot1.setName("moveSmallPivot1");
        this.addAction(moveSmallPivot1);

        KServoAutoAction moveBigPivot2 = new KServoAutoAction(intake.getIntakeBigPivotServo(), IntakeClaw.INTAKE_BIG_PIVOT_INTAKE_READY_POS);
        moveBigPivot2.setName("moveBigPivot2");
        this.addAction(moveBigPivot2);
        moveBigPivot2.setDependentActions(moveSmallPivot1, moveBigPivot1);

        KServoAutoAction moveSmallPivot2 = new KServoAutoAction(intake.getIntakeSmallPivotServo(), IntakeClaw.INTAKE_SMALL_PIVOT_INTAKE_READY_POS);
        moveSmallPivot2.setName("moveSmallPivot2");
        this.addAction(moveSmallPivot2);
        moveSmallPivot2.setDependentActions(moveBigPivot2);

        KServoAutoAction intakeArmBigSweep = new KServoAutoAction(intake.getIntakeBigSweepServo(), IntakeClaw.INTAKE_BIG_SWEEP_PARALLEL_TO_ROBOT);
        intakeArmBigSweep.setName("intakeArmBigSweep");
        this.addAction(intakeArmBigSweep);

        KServoAutoAction moveSmallSweep = new KServoAutoAction(intake.getIntakeSmallSweepServo(), intakeSmallSweepServoPos);
        moveSmallSweep.setName("moveSmallSweep");
//            moveSmallSweep.setDependentActions(moveBigPivot, moveSmallPivot);
        this.addAction(moveSmallSweep);

        KServoAutoAction moveIntakeLSOut = new KServoAutoAction(intake.getIntakeLinkageServo(), IntakeClaw.INTAKE_LINKAGE_OUT_POS);
        moveIntakeLSOut.setName("moveIntakeLSOut");
        this.addAction(moveIntakeLSOut);

    }
}
