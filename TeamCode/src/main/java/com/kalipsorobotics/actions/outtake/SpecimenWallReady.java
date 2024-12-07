package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakeClawAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakePivotAction;
import com.kalipsorobotics.modules.Outtake;

public class SpecimenWallReady extends KActionSet {

    public SpecimenWallReady(Outtake outtake) {
        WaitAction waitOneSec = new WaitAction(1000);
        waitOneSec.setName("waitOneSec");
        this.addAction(waitOneSec);

        MoveOuttakeLSAction lowerSlidesZero = new MoveOuttakeLSAction(outtake, 0);
        lowerSlidesZero.setName("lowerSlidesZero");
        lowerSlidesZero.setDependentActions(waitOneSec);
        this.addAction(lowerSlidesZero);


        KServoAutoAction pivotToWallPos = new KServoAutoAction(outtake.getOuttakePivotServo(),
                OuttakePivotAction.OUTTAKE_PIVOT_WALL_POS);
        pivotToWallPos.setName("pivotToWallPos");
        pivotToWallPos.setDependentActions(waitOneSec);
        this.addAction(pivotToWallPos);

        KServoAutoAction closeClaw = new KServoAutoAction(outtake.getOuttakeClawServo(),
                OuttakeClawAction.OUTTAKE_CLAW_CLOSE_POS);
        closeClaw.setName("closeClaw");
        closeClaw.setDependentActions(lowerSlidesZero);
        this.addAction(closeClaw);
    }

}
