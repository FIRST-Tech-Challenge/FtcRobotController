package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;

public class IntakeTransferReady extends KActionSet {

    public IntakeTransferReady(double intakeLinkageServoPos, IntakeClaw intake, Outtake outtake){

        KServoAutoAction closeClaw = new KServoAutoAction(intake.getIntakeClawServo(), IntakeClaw.INTAKE_CLAW_CLOSE);
        closeClaw.setName("closeClaw");
        this.addAction(closeClaw);

        KServoAutoAction linkageRetract = new KServoAutoAction(intake.getIntakeLinkageServo(), IntakeClaw.INTAKE_LINKAGE_IN_POS);
        linkageRetract.setName("linkageRetract");
        linkageRetract.setDependentActions(closeClaw);
        this.addAction(linkageRetract);

        KServoAutoAction moveBigSweep = new KServoAutoAction(intake.getIntakeBigSweepServo(), IntakeClaw.INTAKE_BIG_SWEEP_TRANSFER_READY_POS);
        moveBigSweep.setName("moveBigSweep");
        moveBigSweep.setDependentActions(closeClaw);
        this.addAction(moveBigSweep);

        KServoAutoAction moveSmallSweep = new KServoAutoAction(intake.getIntakeSmallSweepServo(), IntakeClaw.INTAKE_SMALL_SWEEP_TRANSFER_READY_POS);
        moveSmallSweep.setName("moveSmallSweep");
        moveSmallSweep.setDependentActions(closeClaw);
        this.addAction(moveSmallSweep);

        KServoAutoAction moveBigPivot = new KServoAutoAction(intake.getIntakeBigPivotServo(), IntakeClaw.INTAKE_BIG_PIVOT_TRANSFER_READY_POS);
        moveBigPivot.setName("moveBigPivot");
        moveBigPivot.setDependentActions(closeClaw);
        this.addAction(moveBigPivot);

        KServoAutoAction moveSmallPivot = new KServoAutoAction(intake.getIntakeSmallPivotServo(), IntakeClaw.INTAKE_SMALL_PIVOT_TRANSFER_READY_POS);
        moveSmallPivot.setName("moveSmallPivot");
        moveSmallPivot.setDependentActions(closeClaw);
        this.addAction(moveSmallPivot);


    }
}

//linkage retract
//big sweep parallel to robot (same value as intake ready)
// small sweep perpendicular to robot pos (same value as intake ready)
//big pivot goes to transfer ready pos
//small pivot goes to transfer ready pos
