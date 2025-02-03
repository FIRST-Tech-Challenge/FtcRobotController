package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;

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

        KServoAutoAction closeClaw = new KServoAutoAction(intake.getIntakeClawServo(), IntakeClaw.IntakeClawConfig.INTAKE_CLAW_CLOSE);
        closeClaw.setName("closeClaw");
        closeClaw.setDependentActions(funnelMoveBigPivot, funnelMoveSmallPivot, wait);
        this.addAction(closeClaw);

        KServoAutoAction funnelMoveSmallPivot2 = new KServoAutoAction(intake.getIntakeSmallPivotServo(),
                IntakeClaw.INTAKE_SMALL_PIVOT_FUNNEL_READY_POS);
        funnelMoveSmallPivot2.setName("funnelMoveSmallPivot2");
        this.addAction(funnelMoveSmallPivot2);
        funnelMoveSmallPivot2.setDependentActions(closeClaw);

//        KServoAutoAction unlockRatchet = new KServoAutoAction(intake.getIntakeRatchetServo(),
//                IntakeClaw.INTAKE_RATCHET_UNLOCK_POS);
//        unlockRatchet.setName("unlockRatchet");
//        this.addAction(unlockRatchet);
//
//        KServoAutoAction extendLinkageTransfer = new KServoAutoAction(intake.getIntakeLinkageServo(), IntakeClaw.INTAKE_LINKAGE_SAMPLE_TRANSFER_READY_HALF_POS);
//        extendLinkageTransfer.setName("extendLinkageTransfer");
//        this.addAction(extendLinkageTransfer);
//        extendLinkageTransfer.setDependentActions(closeClaw);


//        KServoAutoAction moveBigPivot2 = new KServoAutoAction(intake.getIntakeBigPivotServo(), IntakeClaw.INTAKE_BIG_PIVOT_TRANSFER_READY_POS);
//        moveBigPivot2.setName("moveBigPivot");
//        moveBigPivot2.setDependentActions(funnelMoveBigPivot, funnelMoveSmallPivot, closeClaw);
//        this.addAction(moveBigPivot2);

    }
}
