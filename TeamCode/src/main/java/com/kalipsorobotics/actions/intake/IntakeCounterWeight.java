package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;

public class IntakeCounterWeight extends KActionSet {

    public IntakeCounterWeight(IntakeClaw intake, Outtake outtake) {

        KServoAutoAction outtakeClawPivotOut = new KServoAutoAction(outtake.getOuttakePivotServo(),
                Outtake.OUTTAKE_PIVOT_HALFWAY_BASKET_POS);
        outtakeClawPivotOut.setName("outtakeClawPivotOut");
        this.addAction(outtakeClawPivotOut);


        KServoAutoAction unlockRatchet = new KServoAutoAction(intake.getIntakeRatchetServo(),
                IntakeClaw.INTAKE_RATCHET_UNLOCK_POS);
        unlockRatchet.setName("unlockRatchet");
        this.addAction(unlockRatchet);


        KServoAutoAction moveBigSweep = new KServoAutoAction(intake.getIntakeBigSweepServo(), IntakeClaw.INTAKE_BIG_SWEEP_TRANSFER_READY_POS);
        moveBigSweep.setName("moveBigSweep");
        this.addAction(moveBigSweep);

        KServoAutoAction moveSmallSweep = new KServoAutoAction(intake.getIntakeSmallSweepServo(), IntakeClaw.INTAKE_SMALL_SWEEP_TRANSFER_READY_POS);
        moveSmallSweep.setName("moveSmallSweep");
        this.addAction(moveSmallSweep);

        KServoAutoAction moveBigPivot = new KServoAutoAction(intake.getIntakeBigPivotServo(), IntakeClaw.INTAKE_BIG_PIVOT_INTAKE_READY_POS);
        moveBigPivot.setName("moveBigPivot");
        //moveBigPivot.setDependentActions(linkageToMid);
        this.addAction(moveBigPivot);

        KServoAutoAction moveSmallPivot = new KServoAutoAction(intake.getIntakeSmallPivotServo(), IntakeClaw.INTAKE_SMALL_PIVOT_INTAKE_READY_POS);
        moveSmallPivot.setName("moveSmallPivot");
        //moveSmallPivot.setDependentActions(linkageToMid);
        this.addAction(moveSmallPivot);

        KServoAutoAction outtakeClawPivotIn = new KServoAutoAction(outtake.getOuttakePivotServo(),
                Outtake.OUTTAKE_PIVOT_TRANSFER_READY_POS);
        outtakeClawPivotIn.setName("outtakeClawPivotIn");
        outtakeClawPivotIn.setDependentActions(moveSmallPivot, moveBigPivot, outtakeClawPivotOut);
        this.addAction(outtakeClawPivotIn);

//        KServoAutoAction linkageExtend = new KServoAutoAction(intake.getIntakeLinkageServo(),
//                IntakeClaw.INTAKE_LINKAGE_SAMPLE_TRANSFER_READY_HALF_POS);
//        linkageExtend.setName("linkageExtend");
//        linkageExtend.setDependentActions(moveBigPivot,moveSmallPivot);
//        this.addAction(linkageExtend);

        KServoAutoAction lockRatchet = new KServoAutoAction(intake.getIntakeRatchetServo(),
                IntakeClaw.INTAKE_RATCHET_LOCK_POS);
        lockRatchet.setName("lockRatchet");
        lockRatchet.setDependentActions(unlockRatchet);
        this.addAction(lockRatchet);

        WaitAction wait = new WaitAction(100);
        wait.setName("wait");
        wait.setDependentActions(moveSmallPivot, moveBigPivot, moveSmallSweep, moveBigSweep);
        this.addAction(wait);

    }
}

//linkage retract
//big sweep parallel to robot (same value as intake ready)
// small sweep perpendicular to robot pos (same value as intake ready)
//big pivot goes to transfer ready pos
//small pivot goes to transfer ready pos
