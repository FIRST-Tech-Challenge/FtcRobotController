package com.kalipsorobotics.actions;

import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.outtake.MoveOuttakeLSAction;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;

public class SampleEndToEndSequence extends KActionSet{
    public SampleEndToEndSequence(IntakeClaw intakeClaw, Outtake outtake){

        KServoAutoAction linkageRetract = new KServoAutoAction(intakeClaw.getIntakeLinkageServo(), IntakeClaw.INTAKE_LINKAGE_IN_POS + 0.03);
        linkageRetract.setName("linkageRetract");
        this.addAction(linkageRetract);

        KServoAutoAction moveBigSweep = new KServoAutoAction(intakeClaw.getIntakeBigSweepServo(), IntakeClaw.INTAKE_BIG_SWEEP_TRANSFER_READY_POS);
        moveBigSweep.setName("moveBigSweep");
        this.addAction(moveBigSweep);

        KServoAutoAction moveSmallSweep = new KServoAutoAction(intakeClaw.getIntakeSmallSweepServo(), IntakeClaw.INTAKE_SMALL_SWEEP_TRANSFER_READY_POS);
        moveSmallSweep.setName("moveSmallSweep");
        this.addAction(moveSmallSweep);

        KServoAutoAction moveBigPivot1 = new KServoAutoAction(intakeClaw.getIntakeBigPivotServo(), IntakeClaw.INTAKE_BIG_PIVOT_TRANSFER_READY_POS);
        moveBigPivot1.setName("moveBigPivot1");
        this.addAction(moveBigPivot1);

        KServoAutoAction moveSmallPivot = new KServoAutoAction(intakeClaw.getIntakeSmallPivotServo(), IntakeClaw.INTAKE_SMALL_PIVOT_TRANSFER_READY_POS);
        moveSmallPivot.setName("moveSmallPivot");
        this.addAction(moveSmallPivot);

        MoveOuttakeLSAction moveLSDown = new MoveOuttakeLSAction(outtake, Outtake.LS_DOWN_POS);
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
        closeOuttakeClaw.setDependentActions(moveBigPivot1, moveSmallPivot, moveSmallSweep, openOuttakeClaw, linkageRetract);
        this.addAction(closeOuttakeClaw);

        KServoAutoAction openIntakeClaw = new KServoAutoAction(intakeClaw.getIntakeClawServo(), IntakeClaw.INTAKE_CLAW_OPEN);
        openIntakeClaw.setName("openIntakeClaw");
        openIntakeClaw.setDependentActions(closeOuttakeClaw);
        this.addAction(openIntakeClaw);

        KServoAutoAction moveBigPivot2 = new KServoAutoAction(intakeClaw.getIntakeBigPivotServo(), IntakeClaw.INTAKE_BIG_PIVOT_INTAKE_READY_POS);
        moveBigPivot2.setName("moveBigPivot2");
        moveBigPivot2.setDependentActions(closeOuttakeClaw, openIntakeClaw);
        this.addAction(moveBigPivot2);

        MoveOuttakeLSAction raiseSlidesBasket = new MoveOuttakeLSAction(outtake, Outtake.LS_SAMPLE_BASKET_READY_POS + 40);
        raiseSlidesBasket.setName("raiseSlidesBasket");
        raiseSlidesBasket.setDependentActions(closeOuttakeClaw, openIntakeClaw);
        this.addAction(raiseSlidesBasket);

        KServoAutoAction pivotOuttakeBasket = new KServoAutoAction(outtake.getOuttakePivotServo(), Outtake.OUTTAKE_PIVOT_BASKET_POS);
        pivotOuttakeBasket.setName("pivotOuttakeBasket");
        pivotOuttakeBasket.setDependentActions(raiseSlidesBasket, closeOuttakeClaw);
        this.addAction(pivotOuttakeBasket);
    }
}
