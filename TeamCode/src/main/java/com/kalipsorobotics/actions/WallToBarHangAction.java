package com.kalipsorobotics.actions;

import com.kalipsorobotics.actions.autoActions.FloorToBarHangRoundTrip;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.actions.outtake.BarHangDistanceSensorAction;
import com.kalipsorobotics.actions.outtake.SpecimenHang;
import com.kalipsorobotics.actions.outtake.SpecimenHangReady;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.Outtake;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

public class WallToBarHangAction extends KActionSet {

    public WallToBarHangAction(DriveTrain driveTrain, WheelOdometry wheelOdometry, Outtake outtake, int barY) {

        WaitAction waitAtStart = new WaitAction(50);
        waitAtStart.setName("waitAtStart");
        this.addAction(waitAtStart);

        PurePursuitAction moveToSpecimenBar = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToSpecimenBar.setName("moveToSpecimenBar");
        moveToSpecimenBar.addPoint(0, 0, 0);
        moveToSpecimenBar.addPoint(-450, barY/2.0, 0, PurePursuitAction.P_XY_FAST, PurePursuitAction.P_ANGLE);
        moveToSpecimenBar.addPoint(FloorToBarHangRoundTrip.SPECIMEN_HANG_POS_X, barY, 0, PurePursuitAction.P_XY_SLOW, PurePursuitAction.P_ANGLE);
        moveToSpecimenBar.setDependentActions(waitAtStart);
        this.addAction(moveToSpecimenBar);

        SpecimenHangReady specimenHangReady1 = new SpecimenHangReady(outtake); // extra hang = -45
        specimenHangReady1.setName("hangSpecimenReady1");
        this.addAction(specimenHangReady1);

//        BarHangDistanceSensorAction barHangDistanceSensorAction = new BarHangDistanceSensorAction(outtake, revdistance, moveToSpecimenBar);
//        barHangDistanceSensorAction.setDependentActions(waitAtStart);
//        this.addAction(barHangDistanceSensorAction);

        SpecimenHang specimenHang = new SpecimenHang(outtake);
        specimenHang.setDependentActions(moveToSpecimenBar, specimenHangReady1);
        this.addAction(specimenHang);

    }
}
