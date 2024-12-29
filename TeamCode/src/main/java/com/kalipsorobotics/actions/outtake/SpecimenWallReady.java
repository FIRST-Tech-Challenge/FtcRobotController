package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.modules.Outtake;

public class SpecimenWallReady extends KActionSet {

    public SpecimenWallReady(Outtake outtake) {
        WaitAction wait = new WaitAction(200);
        wait.setName("wait");
        this.addAction(wait);

        MoveOuttakeLSAction lowerSlidesZero = new MoveOuttakeLSAction(outtake, -5);
        lowerSlidesZero.setName("lowerSlidesZero");
        lowerSlidesZero.setDependentActions(wait);
        this.addAction(lowerSlidesZero);

        KServoAutoAction pivotToWallPos = new KServoAutoAction(outtake.getOuttakePivotServo(), Outtake.OUTTAKE_PIVOT_WALL_READY_POS);
        pivotToWallPos.setName("pivotToWallPos");
        pivotToWallPos.setDependentActions(wait);
        this.addAction(pivotToWallPos);

        KServoAutoAction openClaw = new KServoAutoAction(outtake.getOuttakeClaw(), Outtake.OUTTAKE_CLAW_OPEN);
        openClaw.setName("openClaw");
        openClaw.setDependentActions(lowerSlidesZero);
        this.addAction(openClaw);
    }

}
