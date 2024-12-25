package com.kalipsorobotics.actions;

import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;

public class TransferAction extends KActionSet{
    public TransferAction(IntakeClaw intakeClaw, Outtake outtake) {
        KServoAutoAction closeOuttakeClaw = new KServoAutoAction(outtake.getOuttakeClaw(), Outtake.OUTTAKE_CLAW_CLOSE);
        closeOuttakeClaw.setName("closeOuttakeClaw");
        this.addAction(closeOuttakeClaw);

        KServoAutoAction openIntakeClaw = new KServoAutoAction(intakeClaw.getIntakeClawServo(), IntakeClaw.INTAKE_CLAW_OPEN);
        openIntakeClaw.setName("openIntakeClaw");
        openIntakeClaw.setDependentActions(closeOuttakeClaw);
        this.addAction(openIntakeClaw);

//        KServoAutoAction pullIntake = new KServoAutoAction(intakeClaw.getIntakeLinkageServo(), IntakeClaw.INTAKE_LINKAGE_IN_POS - 0.1);
//        pullIntake.setName("pullIntake");
//        pullIntake.setDependentActions(openIntakeClaw);
//        this.addAction(pullIntake);
    }
}
