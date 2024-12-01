package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.KServoAutoAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakeClawAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakePivotAction;
import com.kalipsorobotics.modules.Outtake;

public class SpecimenWallReady extends KActionSet {

    public SpecimenWallReady(Outtake outtake) {
        WaitAction waitOneSec = new WaitAction(1000);
        waitOneSec.setName("waitOneSec");
        this.addAction(waitOneSec);

        MoveLSAction lowerSlidesZero = new MoveLSAction(outtake, 0);
        lowerSlidesZero.setName("lowerSlidesZero");
        lowerSlidesZero.setDependantActions(waitOneSec);
        this.addAction(lowerSlidesZero);

        KServoAutoAction pivotToWallPos = new KServoAutoAction(outtake.getOuttakePivotServo(),
                OuttakePivotAction.OUTTAKE_PIVOT_WALL_POS);
        pivotToWallPos.setName("pivotToWallPos");
        pivotToWallPos.setDependantActions(waitOneSec);
        this.addAction(pivotToWallPos);

        KServoAutoAction closeClaw = new KServoAutoAction(outtake.getOuttakeClawServo(),
                OuttakeClawAction.OUTTAKE_CLAW_CLOSE_POS);
        closeClaw.setName("closeClaw");
        closeClaw.setDependantActions(lowerSlidesZero);
        this.addAction(closeClaw);
    }

}
