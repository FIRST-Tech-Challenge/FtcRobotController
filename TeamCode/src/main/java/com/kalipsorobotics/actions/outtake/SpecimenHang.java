package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.modules.Outtake;

public class SpecimenHang extends KActionSet {

    public SpecimenHang(Outtake outtake){

        WaitAction waitAction = new WaitAction(10);
        waitAction.setName("waitAction");
        this.addAction(waitAction);

        MoveOuttakeLSAction moveLsToClip = new MoveOuttakeLSAction(outtake, Outtake.LS_SPECIMEN_CLIP_POS);
        moveLsToClip.setName("moveLsToClip");
        moveLsToClip.setDependentActions(waitAction);
        this.addAction(moveLsToClip);

        KServoAutoAction clawOpen = new KServoAutoAction(outtake.getOuttakeClaw(), Outtake.OUTTAKE_CLAW_OPEN);
        clawOpen.setName("clawOpen");
        clawOpen.setDependentActions(moveLsToClip);
        this.addAction(clawOpen);
    }
}
