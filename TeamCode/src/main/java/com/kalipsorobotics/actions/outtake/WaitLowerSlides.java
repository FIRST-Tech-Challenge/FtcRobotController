package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.KServoAutoAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakePivotAction;
import com.kalipsorobotics.modules.Outtake;

public class WaitLowerSlides extends KActionSet {

    public WaitLowerSlides(Outtake outtake) {
        WaitAction waitLowerSlides1 = new WaitAction(1000);
        waitLowerSlides1.setName("waitLowerSlides1");
        this.addAction(waitLowerSlides1);

        MoveLSAction lowerSlidesZero1 = new MoveLSAction(outtake, 0);
        lowerSlidesZero1.setName("lowerSlidesZero1");
        lowerSlidesZero1.setDependantActions(waitLowerSlides1);
        this.addAction(lowerSlidesZero1);

        KServoAutoAction pivotToWallPos = new KServoAutoAction(outtake.getOuttakePivotServo(),
                OuttakePivotAction.OUTTAKE_PIVOT_WALL_POS);
        pivotToWallPos.setName("pivotToWallPos");
        this.addAction(pivotToWallPos);

    }

}
