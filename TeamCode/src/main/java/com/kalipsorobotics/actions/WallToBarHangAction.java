package com.kalipsorobotics.actions;

import com.kalipsorobotics.actions.autoActions.FloorToBarHangRoundTrip;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.actions.outtake.SpecimenHang;
import com.kalipsorobotics.actions.outtake.SpecimenHangReady;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.Outtake;

public class WallToBarHangAction extends KActionSet {

    public WallToBarHangAction(DriveTrain driveTrain, WheelOdometry wheelOdometry, Outtake outtake, int barY) {

        WaitAction waitAtStart = new WaitAction(100);
        waitAtStart.setName("waitAtStart");
        this.addAction(waitAtStart);

        PurePursuitAction moveToSpecimenBar = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToSpecimenBar.setName("moveToSpecimenBar");
        moveToSpecimenBar.addPoint(0, 0, 0);
        moveToSpecimenBar.addPoint(FloorToBarHangRoundTrip.SPECIMEN_HANG_POS_X+15, barY, 0);
        moveToSpecimenBar.setMaxCheckDoneCounter(5);
        moveToSpecimenBar.setDependentActions(waitAtStart);
        this.addAction(moveToSpecimenBar);

        SpecimenHangReady specimenHangReady1 = new SpecimenHangReady(outtake);
        specimenHangReady1.setName("hangSpecimenReady1");
        this.addAction(specimenHangReady1);

        SpecimenHang specimenHang1 = new SpecimenHang(outtake);
        specimenHang1.setName("specimenHang1");
        specimenHang1.setDependentActions(specimenHangReady1, moveToSpecimenBar);
        this.addAction(specimenHang1);


    }
}
