package com.kalipsorobotics.actions;

import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.outtake.MoveLSAction;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;

public class TrussSpecimenEndToEndSequence extends KActionSet{
    public TrussSpecimenEndToEndSequence(IntakeClaw intakeClaw, Outtake outtake){
        KServoAutoAction ratchetLock = new KServoAutoAction(intakeClaw.getIntakeRatchetServo(), IntakeClaw.INTAKE_RATCHET_LOCK_POS);
        ratchetLock.setName("ratchetLock");
        this.addAction(ratchetLock);

        KServoAutoAction moveBigSweep = new KServoAutoAction(intakeClaw.getIntakeBigSweepServo(), IntakeClaw.INTAKE_BIG_SWEEP_TRANSFER_READY_POS);
        moveBigSweep.setName("moveBigSweep");
        this.addAction(moveBigSweep);

        KServoAutoAction moveSmallSweep = new KServoAutoAction(intakeClaw.getIntakeSmallSweepServo(), IntakeClaw.INTAKE_SMALL_SWEEP_TRANSFER_READY_POS);
        moveSmallSweep.setName("moveSmallSweep");
        this.addAction(moveSmallSweep);

//        WaitAction waitMoveSmallPivotHeadStart = new WaitAction(200);
//        waitMoveSmallPivotHeadStart.setName("waitMoveSmallPivotHeadStart");
//        this.addAction(waitMoveSmallPivotHeadStart);

        KServoAutoAction moveSmallPivot1 = new KServoAutoAction(intakeClaw.getIntakeSmallPivotServo(),
                IntakeClaw.INTAKE_SMALL_PIVOT_TRANSFER_UNDER_RUNG_MIDDLE_MIDDLE_POS);
        moveSmallPivot1.setName("moveSmallPivot1");
        this.addAction(moveSmallPivot1);

        //moveSmallPivot1.setDependentActions(waitMoveSmallPivotHeadStart);
//        moveSmallPivot1.setDependentActions(moveBigPivot1);

        KServoAutoAction moveBigPivot1 = new KServoAutoAction(intakeClaw.getIntakeBigPivotServo(),
                IntakeClaw.INTAKE_BIG_PIVOT_TRANSFER_UNDER_RUNG_MIDDLE_POS);
        moveBigPivot1.setName("moveBigPivot1");
        this.addAction(moveBigPivot1);
        moveBigPivot1.setDependentActions(moveSmallPivot1);


        KServoAutoAction moveSmallPivot2 = new KServoAutoAction(intakeClaw.getIntakeSmallPivotServo(),
                IntakeClaw.INTAKE_SMALL_PIVOT_TRANSFER_UNDER_RUNG_MIDDLE_POS);
        moveSmallPivot2.setName("moveSmallPivot2");
        this.addAction(moveSmallPivot2);
        moveSmallPivot2.setDependentActions(moveSmallPivot1);

        KServoAutoAction moveBigPivot2 = new KServoAutoAction(intakeClaw.getIntakeBigPivotServo(),
                IntakeClaw.INTAKE_BIG_PIVOT_TRANSFER_READY_POS);
        moveBigPivot2.setName("moveBigPivot2");
        this.addAction(moveBigPivot2);
        moveBigPivot2.setDependentActions(moveSmallPivot2, moveBigPivot1);

        KServoAutoAction linkageRetractHalf1 = new KServoAutoAction(intakeClaw.getIntakeLinkageServo(),
                IntakeClaw.INTAKE_LINKAGE_SAMPLE_TRANSFER_READY_HALF_POS);
        linkageRetractHalf1.setName("linkageRetract");
        this.addAction(linkageRetractHalf1);
        linkageRetractHalf1.setDependentActions(moveBigPivot1);

        KServoAutoAction moveSmallPivot3 = new KServoAutoAction(intakeClaw.getIntakeSmallPivotServo(),
                IntakeClaw.INTAKE_SMALL_PIVOT_TRANSFER_READY_POS);
        moveSmallPivot3.setName("moveSmallPivot3");
        this.addAction(moveSmallPivot3);
        moveSmallPivot3.setDependentActions(linkageRetractHalf1);

        KServoAutoAction linkageRetractFull1 = new KServoAutoAction(intakeClaw.getIntakeLinkageServo(),
                IntakeClaw.INTAKE_LINKAGE_IN_POS);
        linkageRetractFull1.setName("linkageRetractFull1");
        this.addAction(linkageRetractFull1);
        linkageRetractFull1.setDependentActions(moveSmallPivot3);

        MoveLSAction moveLSDown = new MoveLSAction(outtake, Outtake.LS_DOWN_POS);
        moveLSDown.setName("moveLSDown");
        this.addAction(moveLSDown);

        KServoAutoAction moveOuttakeTransferReady = new KServoAutoAction(outtake.getOuttakePivotServo(), Outtake.OUTTAKE_PIVOT_TRANSFER_READY_POS);
        moveOuttakeTransferReady.setName("moveOuttakeTransferReady");
        this.addAction(moveOuttakeTransferReady);

        KServoAutoAction openOuttakeClaw = new KServoAutoAction(outtake.getOuttakeClaw(), Outtake.OUTTAKE_CLAW_OPEN);
        openOuttakeClaw.setName("openOuttakeClaw");
        this.addAction(openOuttakeClaw);

        KServoAutoAction closeOuttakeClaw = new KServoAutoAction(outtake.getOuttakeClaw(), Outtake.OUTTAKE_CLAW_CLOSE);
        closeOuttakeClaw.setName("closeOuttakeClaw");
        closeOuttakeClaw.setDependentActions(moveBigPivot1, moveSmallPivot1, moveSmallSweep, openOuttakeClaw,
                linkageRetractFull1);
        this.addAction(closeOuttakeClaw);

        KServoAutoAction openIntakeClaw = new KServoAutoAction(intakeClaw.getIntakeClawServo(), IntakeClaw.INTAKE_CLAW_OPEN);
        openIntakeClaw.setName("openIntakeClaw");
        openIntakeClaw.setDependentActions(closeOuttakeClaw);
        this.addAction(openIntakeClaw);

//        KServoAutoAction moveBigPivot2 = new KServoAutoAction(intakeClaw.getIntakeBigPivotServo(), IntakeClaw.INTAKE_BIG_PIVOT_INTAKE_READY_POS);
//        moveBigPivot2.setName("moveBigPivot2");
//        moveBigPivot2.setDependentActions(closeOuttakeClaw, openIntakeClaw);
//        this.addAction(moveBigPivot2);

        KServoAutoAction pivotOuttakeBasket = new KServoAutoAction(outtake.getOuttakePivotServo(), Outtake.OUTTAKE_PIVOT_BASKET_POS);
        pivotOuttakeBasket.setName("pivotOuttakeBasket");
        pivotOuttakeBasket.setDependentActions(closeOuttakeClaw);
        this.addAction(pivotOuttakeBasket);

    }
}
