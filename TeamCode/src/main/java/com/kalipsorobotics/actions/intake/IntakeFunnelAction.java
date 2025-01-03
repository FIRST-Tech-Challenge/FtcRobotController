package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.outtake.OuttakeTransferReady;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;

import org.checkerframework.checker.units.qual.K;

public class IntakeFunnelAction extends KActionSet {

    public IntakeFunnelAction(IntakeClaw intake, Outtake outtake) {

        KServoAutoAction funnelMoveBigPivot = new KServoAutoAction(intake.getIntakeBigPivotServo(), IntakeClaw.INTAKE_BIG_PIVOT_FUNNEL_GRAB_POS);
        funnelMoveBigPivot.setName("moveBigPivot");
        this.addAction(funnelMoveBigPivot);

        KServoAutoAction funnelMoveSmallPivot = new KServoAutoAction(intake.getIntakeSmallPivotServo(), IntakeClaw.INTAKE_SMALL_PIVOT_FUNNEL_GRAB_POS);
        funnelMoveSmallPivot.setName("funnelMoveSmallPivot");
        this.addAction(funnelMoveSmallPivot);

        WaitAction wait = new WaitAction(100);
        wait.setName("wait");
        this.addAction(wait);

        KServoAutoAction closeClaw = new KServoAutoAction(intake.getIntakeClawServo(), IntakeClaw.INTAKE_CLAW_CLOSE);
        closeClaw.setName("closeClaw");
        closeClaw.setDependentActions(funnelMoveBigPivot, funnelMoveSmallPivot, wait);
        this.addAction(closeClaw);

//        KServoAutoAction moveBigPivot2 = new KServoAutoAction(intake.getIntakeBigPivotServo(), IntakeClaw.INTAKE_BIG_PIVOT_TRANSFER_READY_POS);
//        moveBigPivot2.setName("moveBigPivot");
//        moveBigPivot2.setDependentActions(funnelMoveBigPivot, funnelMoveSmallPivot, closeClaw);
//        this.addAction(moveBigPivot2);

    }
}
