package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.modules.Outtake;

public class SpecimanHang extends KActionSet {

    public SpecimanHang (Outtake outtake){
//        KServoAutoAction pivotOuttake = new KServoAutoAction(outtake.getOuttakePivotServo(), Outtake.OUTTAKE_PIVOT_SPECIMAN_HANG_POS);
//        pivotOuttake.setName("pivotOuttake");
//        this.addAction(pivotOuttake);

        WaitAction waitAction = new WaitAction(10);
        waitAction.setName("waitAction");
//        waitAction.setDependentActions(pivotOuttake);
        this.addAction(waitAction);

        MoveOuttakeLSAction moveLsToClip = new MoveOuttakeLSAction(outtake, Outtake.LS_SPECIMAN_CLIP_POS);
        moveLsToClip.setName("moveLsToClip");
        moveLsToClip.setDependentActions(waitAction);
        this.addAction(moveLsToClip);

        KServoAutoAction clawOpen = new KServoAutoAction(outtake.getOuttakeClaw(), Outtake.OUTTAKE_CLAW_OPEN);
        clawOpen.setName("clawOpen");
        clawOpen.setDependentActions(moveLsToClip);
        this.addAction(clawOpen);

        MoveOuttakeLSAction moveLsDown = new MoveOuttakeLSAction(outtake, Outtake.LS_DOWN_POS);
        moveLsDown.setName("moveLsDown");
        moveLsDown.setDependentActions(clawOpen);
        this.addAction(moveLsDown);
    }
}
