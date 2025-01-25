package com.kalipsorobotics.actions;

import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;

public class TransferAction extends KActionSet{
    public TransferAction(IntakeClaw intakeClaw, Outtake outtake) {
        KServoAutoAction lockRatchet = new KServoAutoAction(intakeClaw.getIntakeRatchetServo(), IntakeClaw.INTAKE_RATCHET_LOCK_POS);
        lockRatchet.setName("lockRatchet");
        this.addAction(lockRatchet);


        KServoAutoAction closeOuttakeClaw = new KServoAutoAction(outtake.getOuttakeClaw(), Outtake.OUTTAKE_CLAW_CLOSE);
        closeOuttakeClaw.setName("closeOuttakeClaw");
        this.addAction(closeOuttakeClaw);
        setDependentActions(lockRatchet);

        WaitAction wait = new WaitAction(50);
        wait.setName("wait");
        wait.setDependentActions(closeOuttakeClaw);
        this.addAction(wait);

        KServoAutoAction openIntakeClaw = new KServoAutoAction(intakeClaw.getIntakeClawServo(), IntakeClaw.INTAKE_CLAW_OPEN);
        openIntakeClaw.setName("openIntakeClaw");
        openIntakeClaw.setDependentActions(wait);
        this.addAction(openIntakeClaw);

        WaitAction wait2 = new WaitAction(150);
        wait2.setName("wait2");
        wait2.setDependentActions(openIntakeClaw);
        this.addAction(wait2);
//        KServoAutoAction moveBigPivot = new KServoAutoAction(intakeClaw.getIntakeBigPivotServo(), IntakeClaw.INTAKE_BIG_PIVOT_INTAKE_READY_POS);
//        moveBigPivot.setName("moveBigPivot");
//        moveBigPivot.setDependentActions(closeOuttakeClaw,openIntakeClaw);
//        this.addAction(moveBigPivot);

//        KServoAutoAction pullIntake = new KServoAutoAction(intakeClaw.getIntakeLinkageServo(), IntakeClaw.INTAKE_LINKAGE_IN_POS - 0.1);
//        pullIntake.setName("pullIntake");
//        pullIntake.setDependentActions(openIntakeClaw);
//        this.addAction(pullIntake);
    }
}
