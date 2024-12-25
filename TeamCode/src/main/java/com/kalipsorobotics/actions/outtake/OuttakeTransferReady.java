package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;

public class OuttakeTransferReady extends KActionSet {
    public OuttakeTransferReady(double intakeLinkageServoPos, Outtake outtake){
        MoveOuttakeLSAction moveLSDown = new MoveOuttakeLSAction(outtake, Outtake.LS_DOWN_POS);
        moveLSDown.setName("moveLSDown");
        this.addAction(moveLSDown);

        KServoAutoAction clawOpen = new KServoAutoAction(outtake.outtakeClawServo, Outtake.OUTTAKE_CLAW_OPEN);
        clawOpen.setName("clawOpen");
        this.addAction(clawOpen);

        KServoAutoAction moveOuttakePivot = new KServoAutoAction(outtake.outtakePivotServo, Outtake.OUTTAKE_PIVOT_TRANSFER_READY_POS);
        moveOuttakePivot.setName("moveOuttakePivot");
        this.addAction(moveOuttakePivot);
    }
}
//ls move down
//claw open
//outtake pivot goes to transfer pos

