package com.kalipsorobotics.actions.outtake;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakeClawAction;
import com.kalipsorobotics.modules.Outtake;

public class OuttakeDownReady extends KActionSet {

    public OuttakeDownReady(Outtake outtake) {

        WaitAction wait100 = new WaitAction(100);
        wait100.setName("wait100");
        this.addAction(wait100);

        KServoAutoAction outtakePivotActionClose = new KServoAutoAction(outtake.getOuttakePivotServo(),
                Outtake.OUTTAKE_PIVOT_SPECIMAN_HANG_READY_POS);
        outtakePivotActionClose.setName("outtakePivotActionCloseOuttakeReady");
        this.addAction(outtakePivotActionClose);

        KServoAutoAction outtakeClawActionOpen = new KServoAutoAction(outtake.getOuttakeClaw(),
                OuttakeClawAction.OUTTAKE_CLAW_OPEN_POS);
        outtakeClawActionOpen.setName("outtakeClawActionOpenOuttakeReady");
        this.addAction(outtakeClawActionOpen);

        MoveOuttakeLSAction lowerSlides = new MoveOuttakeLSAction(outtake, 100);
        lowerSlides.setName("lowerSlidesOuttakeReady");
        this.addAction(lowerSlides);
    }
}
