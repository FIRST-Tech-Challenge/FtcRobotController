package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.modules.IntakeClaw;

public class IntakeFunnelAction extends KActionSet {

    public IntakeFunnelAction(IntakeClaw intake) {

        KServoAutoAction moveBigPivot = new KServoAutoAction(intake.getIntakeBigSweepServo(), IntakeClaw.INTAKE_BIG_PIVOT_FUNNEL_READY_POS);
        moveBigPivot.setName("moveBigPivot");
        this.addAction(moveBigPivot);

        KServoAutoAction funnelMoveSmallPivot = new KServoAutoAction(intake.getIntakeBigSweepServo(), IntakeClaw.INTAKE_SMALL_PIVOT_FUNNEL_GRAB_POS);
        funnelMoveSmallPivot.setName("funnelMoveSmallPivot");
        this.addAction(funnelMoveSmallPivot);

        KServoAutoAction funnelMoveBigPivot = new KServoAutoAction(intake.getIntakeBigSweepServo(), IntakeClaw.INTAKE_BIG_PIVOT_FUNNEL_GRAB_POS);
        funnelMoveBigPivot.setName("funnelMoveBigPivot");
        this.addAction(funnelMoveBigPivot);

        KServoAutoAction openClaw = new KServoAutoAction(intake.getIntakeClawServo(), IntakeClaw.INTAKE_CLAW_CLOSE);
        openClaw.setName("openClaw");
        this.addAction(openClaw);



    }



}
