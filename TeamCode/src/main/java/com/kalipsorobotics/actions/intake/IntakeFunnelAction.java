package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.modules.IntakeClaw;

public class IntakeFunnelAction extends KActionSet {

    public IntakeFunnelAction(IntakeClaw intake) {

        KServoAutoAction funnelMoveBigPivot = new KServoAutoAction(intake.getIntakeBigSweepServo(), IntakeClaw.INTAKE_BIG_PIVOT_TRANSFER_READY_POS);
        funnelMoveBigPivot.setName("moveBigPivot");
        this.addAction(funnelMoveBigPivot);

        KServoAutoAction funnelMoveSmallPivot = new KServoAutoAction(intake.getIntakeBigSweepServo(), IntakeClaw.INTAKE_SMALL_PIVOT_FUNNEL_GRAB_POS);
        funnelMoveSmallPivot.setName("funnelMoveSmallPivot");
        this.addAction(funnelMoveBigPivot);


        KServoAutoAction closeClaw = new KServoAutoAction(intake.getIntakeClawServo(), IntakeClaw.INTAKE_CLAW_CLOSE);
        closeClaw.setName("openClaw");
        closeClaw.setDependentActions(funnelMoveSmallPivot, funnelMoveBigPivot);
        this.addAction(closeClaw);

        KServoAutoAction moveBigPivot2 = new KServoAutoAction(intake.getIntakeBigSweepServo(), IntakeClaw.INTAKE_BIG_PIVOT_TRANSFER_READY_POS);
        moveBigPivot2.setName("moveBigPivot");
        moveBigPivot2.setDependentActions(funnelMoveSmallPivot, funnelMoveBigPivot, closeClaw);
        this.addAction(moveBigPivot2);

    }
}
