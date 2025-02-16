package com.kalipsorobotics.actions;

import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.modules.IntakeClaw;

public class SampleSweepingReady extends KActionSet{

    public SampleSweepingReady(IntakeClaw intake){

        KServoAutoAction pushRatchet = new KServoAutoAction(intake.getIntakeRatchetServo(), IntakeClaw.INTAKE_RATCHET_UNLOCK_POS);
        pushRatchet.setName("pushRatchet");
        this.addAction(pushRatchet);

        KServoAutoAction moveOpenClaw = new KServoAutoAction(intake.getIntakeClawServo(), IntakeClaw.IntakeClawConfig.INTAKE_CLAW_OPEN);
        moveOpenClaw.setName("moveOpenClaw");
        this.addAction(moveOpenClaw);

        KServoAutoAction moveBigPivot2 = new KServoAutoAction(intake.getIntakeBigPivotServo(), IntakeClaw.INTAKE_BIG_PIVOT_SWEEPING_READY);
        moveBigPivot2.setName("moveBigPivot2");
        this.addAction(moveBigPivot2);

        KServoAutoAction moveSmallPivot2 = new KServoAutoAction(intake.getIntakeSmallPivotServo(), IntakeClaw.INTAKE_SMALL_PIVOT_SWEEPING_READY);
        moveSmallPivot2.setName("moveSmallPivot2");
        this.addAction(moveSmallPivot2);
        moveSmallPivot2.setDependentActions(moveBigPivot2);

        KServoAutoAction intakeArmBigSweep = new KServoAutoAction(intake.getIntakeBigSweepServo(), IntakeClaw.INTAKE_BIG_SWEEP_PARALLEL_TO_ROBOT);
        intakeArmBigSweep.setName("intakeArmBigSweep");
        this.addAction(intakeArmBigSweep);

        KServoAutoAction moveSmallSweep = new KServoAutoAction(intake.getIntakeSmallSweepServo(), IntakeClaw.INTAKE_SMALL_SWEEP_SWEEPING_READY);
        moveSmallSweep.setName("moveSmallSweep");
        this.addAction(moveSmallSweep);

        KServoAutoAction moveIntakeLSOut = new KServoAutoAction(intake.getIntakeLinkageServo(), IntakeClaw.INTAKE_LINKAGE_OUT_POS);
        moveIntakeLSOut.setName("moveIntakeLSOut");
        this.addAction(moveIntakeLSOut);

    }
}
