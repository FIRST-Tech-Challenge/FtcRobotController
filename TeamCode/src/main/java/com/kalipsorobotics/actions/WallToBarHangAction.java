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

    public WallToBarHangAction(DriveTrain driveTrain, WheelOdometry wheelOdometry, Outtake outtake, Rev2mDistanceSensor revdistance, int barY) {

        WaitAction waitAtStart = new WaitAction(100);
        waitAtStart.setName("waitAtStart");
        this.addAction(waitAtStart);

        PurePursuitAction moveToSpecimenBar = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToSpecimenBar.setName("moveToSpecimenBar");
        moveToSpecimenBar.addPoint(0, 0, 0);
        moveToSpecimenBar.addPoint(FloorToBarHangRoundTrip.SPECIMEN_HANG_POS_X, barY, 0);
        moveToSpecimenBar.setMaxCheckDoneCounter(10);
        moveToSpecimenBar.setDependentActions(waitAtStart);
        this.addAction(moveToSpecimenBar);

        SpecimenHangReady specimenHangReady1 = new SpecimenHangReady(outtake);
        specimenHangReady1.setName("hangSpecimenReady1");
        this.addAction(specimenHangReady1);

        BarHangDistanceSensorAction barHangDistanceSensorAction = new BarHangDistanceSensorAction(outtake, revdistance, moveToSpecimenBar);
        barHangDistanceSensorAction.setDependentActions(waitAtStart);
        this.addAction(barHangDistanceSensorAction);

        SpecimenHang specimenHang = new SpecimenHang(outtake);
        specimenHang.setDependentActions(barHangDistanceSensorAction);
        this.addAction(specimenHang);

    }
}
