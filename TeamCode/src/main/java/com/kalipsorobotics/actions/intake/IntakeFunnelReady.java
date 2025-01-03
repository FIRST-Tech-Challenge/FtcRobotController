package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.modules.IntakeClaw;

public class IntakeFunnelReady extends KActionSet {
    public IntakeFunnelReady(IntakeClaw intake) {

        KServoAutoAction moveSmallSweep = new KServoAutoAction(intake.getIntakeSmallSweepServo(), IntakeClaw.INTAKE_SMALL_SWEEP_TRANSFER_READY_POS);
        moveSmallSweep.setName("moveSmallSweep");
        this.addAction(moveSmallSweep);

        KServoAutoAction linkageRetract = new KServoAutoAction(intake.getIntakeLinkageServo(), IntakeClaw.INTAKE_LINKAGE_IN_POS);
        linkageRetract.setName("linkageRetract");
        this.addAction(linkageRetract);

        KServoAutoAction openClaw = new KServoAutoAction(intake.getIntakeClawServo(), IntakeClaw.INTAKE_CLAW_OPEN);
        openClaw.setName("openClaw");
        this.addAction(openClaw);

        KServoAutoAction moveBigPivot = new KServoAutoAction(intake.getIntakeBigSweepServo(), IntakeClaw.INTAKE_BIG_PIVOT_FUNNEL_READY_POS);
        moveBigPivot.setName("moveBigPivot");
        this.addAction(moveBigPivot);

        KServoAutoAction moveSmallPivot = new KServoAutoAction(intake.getIntakeBigSweepServo(), IntakeClaw.INTAKE_SMALL_PIVOT_FUNNEL_READY_POS);
        moveSmallPivot.setName("moveSmallPivot");
        this.addAction(moveSmallPivot);




    }



}
