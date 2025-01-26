package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;

import org.checkerframework.checker.units.qual.K;

public class IntakeTransferReady extends KActionSet {

    public IntakeTransferReady(IntakeClaw intake) {

        KServoAutoAction lockRatchet = new KServoAutoAction(intake.getIntakeRatchetServo(), IntakeClaw.INTAKE_RATCHET_LOCK_POS);
        lockRatchet.setName("lockRatchet");
        this.addAction(lockRatchet);

        KServoAutoAction closeClaw = new KServoAutoAction(intake.getIntakeClawServo(), IntakeClaw.INTAKE_CLAW_CLOSE);
        closeClaw.setName("closeClaw");
        this.addAction(closeClaw);

        KServoAutoAction linkageToMid = new KServoAutoAction(intake.getIntakeLinkageServo(), IntakeClaw.INTAKE_LINKAGE_MID_POS);
        linkageToMid.setName("linkageToMid");
        linkageToMid.setDependentActions(closeClaw);
        this.addAction(linkageToMid);

        KServoAutoAction linkageRetract = new KServoAutoAction(intake.getIntakeLinkageServo(), IntakeClaw.INTAKE_LINKAGE_IN_POS);
        linkageRetract.setName("linkageRetract");
        linkageRetract.setDependentActions(linkageToMid);
        this.addAction(linkageRetract);

        KServoAutoAction moveBigSweep = new KServoAutoAction(intake.getIntakeBigSweepServo(), IntakeClaw.INTAKE_BIG_SWEEP_TRANSFER_READY_POS);
        moveBigSweep.setName("moveBigSweep");
        moveBigSweep.setDependentActions(closeClaw);
        this.addAction(moveBigSweep);

        KServoAutoAction moveSmallSweep = new KServoAutoAction(intake.getIntakeSmallSweepServo(), IntakeClaw.INTAKE_SMALL_SWEEP_TRANSFER_READY_POS);
        moveSmallSweep.setName("moveSmallSweep");
        moveSmallSweep.setDependentActions(closeClaw);
        this.addAction(moveSmallSweep);

        KServoAutoAction moveSmallPivot = new KServoAutoAction(intake.getIntakeSmallPivotServo(), IntakeClaw.INTAKE_SMALL_PIVOT_TRANSFER_READY_POS);
        moveSmallPivot.setName("moveSmallPivot");
        moveSmallPivot.setDependentActions(closeClaw);
        this.addAction(moveSmallPivot);

        KServoAutoAction moveBigPivot = new KServoAutoAction(intake.getIntakeBigPivotServo(), IntakeClaw.INTAKE_BIG_PIVOT_TRANSFER_READY_POS);
        moveBigPivot.setName("moveBigPivot");
        moveBigPivot.setDependentActions(moveSmallPivot);
        this.addAction(moveBigPivot);


        WaitAction wait = new WaitAction(100);
        wait.setName("wait");
        wait.setDependentActions(moveSmallPivot, moveBigPivot, moveSmallSweep, moveBigSweep, linkageRetract);
        this.addAction(wait);

    }
}

//linkage retract
//big sweep parallel to robot (same value as intake ready)
// small sweep perpendicular to robot pos (same value as intake ready)
//big pivot goes to transfer ready pos
//small pivot goes to transfer ready pos
