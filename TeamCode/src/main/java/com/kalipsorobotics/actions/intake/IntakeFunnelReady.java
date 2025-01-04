package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;

public class IntakeFunnelReady extends KActionSet {

    public IntakeFunnelReady(IntakeClaw intake, Outtake outtake){
        KServoAutoAction moveOuttakePivot = new KServoAutoAction(outtake.getOuttakePivotServo(), Outtake.OUTTAKE_PIVOT_HALFWAY_BASKET_POS);
        moveOuttakePivot.setName("moveOuttakePivot");
        this.addAction(moveOuttakePivot);

        KServoAutoAction moveBigPivot = new KServoAutoAction(intake.getIntakeBigPivotServo(), IntakeClaw.INTAKE_BIG_PIVOT_FUNNEL_READY_POS);
        moveBigPivot.setName("moveBigPivot");
        moveBigPivot.setDependentActions(moveOuttakePivot);
        this.addAction(moveBigPivot);

        KServoAutoAction moveSmallPivot = new KServoAutoAction(intake.getIntakeSmallPivotServo(), IntakeClaw.INTAKE_SMALL_PIVOT_FUNNEL_READY_POS);
        moveSmallPivot.setName("moveSmallPivot");
        moveSmallPivot.setDependentActions(moveOuttakePivot);
        this.addAction(moveSmallPivot);

        KServoAutoAction moveSmallSweep = new KServoAutoAction(intake.getIntakeSmallSweepServo(), IntakeClaw.INTAKE_SMALL_SWEEP_TRANSFER_READY_POS);
        moveSmallSweep.setName("moveSmallSweep");
        moveSmallSweep.setDependentActions(moveOuttakePivot);
        this.addAction(moveSmallSweep);

        KServoAutoAction openClaw = new KServoAutoAction(intake.getIntakeClawServo(), IntakeClaw.INTAKE_CLAW_OPEN);
        openClaw.setName("openClaw");
        this.addAction(openClaw);

        KServoAutoAction linkageRetract = new KServoAutoAction(intake.getIntakeLinkageServo(), IntakeClaw.INTAKE_LINKAGE_IN_POS);
        linkageRetract.setName("linkageRetract");
        this.addAction(linkageRetract);
    }

    public IntakeFunnelReady(IntakeClaw intake, Outtake outtake, boolean movePivot) {

        KServoAutoAction moveBigPivot = new KServoAutoAction(intake.getIntakeBigPivotServo(), IntakeClaw.INTAKE_BIG_PIVOT_FUNNEL_READY_POS);
        moveBigPivot.setName("moveBigPivot");
        this.addAction(moveBigPivot);

        KServoAutoAction moveSmallPivot = new KServoAutoAction(intake.getIntakeSmallPivotServo(), IntakeClaw.INTAKE_SMALL_PIVOT_FUNNEL_READY_POS);
        moveSmallPivot.setName("moveSmallPivot");
        this.addAction(moveSmallPivot);

        KServoAutoAction moveSmallSweep = new KServoAutoAction(intake.getIntakeSmallSweepServo(), IntakeClaw.INTAKE_SMALL_SWEEP_TRANSFER_READY_POS);
        moveSmallSweep.setName("moveSmallSweep");
        this.addAction(moveSmallSweep);

        KServoAutoAction openClaw = new KServoAutoAction(intake.getIntakeClawServo(), IntakeClaw.INTAKE_CLAW_OPEN);
        openClaw.setName("openClaw");
        this.addAction(openClaw);

        KServoAutoAction linkageRetract = new KServoAutoAction(intake.getIntakeLinkageServo(), IntakeClaw.INTAKE_LINKAGE_IN_POS);
        linkageRetract.setName("linkageRetract");
        this.addAction(linkageRetract);

    }
}
