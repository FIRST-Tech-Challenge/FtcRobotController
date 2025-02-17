package com.kalipsorobotics.actions.autoActions;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.MoveToDistanceThreshold;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.outtake.DistanceDetectionAction;
import com.kalipsorobotics.actions.outtake.SpecimenHang;
import com.kalipsorobotics.actions.outtake.SpecimenHangReady;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.Outtake;

public class FirstWallToBarHangAction extends KActionSet {

    public FirstWallToBarHangAction(DriveTrain driveTrain, WheelOdometry wheelOdometry, Outtake outtake, int barY) {
        this(driveTrain, wheelOdometry, outtake, barY, false);
    }

    public FirstWallToBarHangAction(DriveTrain driveTrain, WheelOdometry wheelOdometry, Outtake outtake, int barY,
                                    boolean isSlow) {

        WaitAction waitAtStart = new WaitAction(50);
        waitAtStart.setName("waitAtStart");
        this.addAction(waitAtStart);

        PurePursuitAction moveToSpecimenBar = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToSpecimenBar.setName("moveToSpecimenBar");
            moveToSpecimenBar.addPoint(-450, barY/2.0 + 50, 0,
                    isSlow?PurePursuitAction.P_XY : PurePursuitAction.P_XY_FAST,
                    PurePursuitAction.P_ANGLE);
        moveToSpecimenBar.addPoint(FloorToBarHangRoundTrip.SPECIMEN_HANG_POS_X, barY, 0, PurePursuitAction.P_XY_SLOW, PurePursuitAction.P_ANGLE);
        moveToSpecimenBar.setDependentActions(waitAtStart);
        this.addAction(moveToSpecimenBar);

        SpecimenHangReady specimenHangReady1 = new SpecimenHangReady(outtake); // extra hang = -45
        specimenHangReady1.setName("hangSpecimenReady1");
        this.addAction(specimenHangReady1);

//        PurePursuitDistanceSensorCorrection purePursuitDistanceSensorCorrection =
//                new PurePursuitDistanceSensorCorrection(outtake, moveToSpecimenBar, driveTrain);
//        purePursuitDistanceSensorCorrection.setName("barHangPickupDistanceSensorActionMoveUntilThreshold");
//        purePursuitDistanceSensorCorrection.setDependentActions(specimenHangReady1);
//        this.addAction(purePursuitDistanceSensorCorrection);

        DistanceDetectionAction distanceDetectionAction = new DistanceDetectionAction(outtake.getRevDistanceBottom(),
                155);
        distanceDetectionAction.setName("distanceDetectionAction");
        distanceDetectionAction.setDependentActions(moveToSpecimenBar);
        this.addAction(distanceDetectionAction);

        MoveToDistanceThreshold moveToDistanceThreshold = new MoveToDistanceThreshold(driveTrain,
                distanceDetectionAction, -0.3, 1000);
        moveToDistanceThreshold.setName("moveToDistanceThreshold");
        moveToDistanceThreshold.setDependentActions(moveToSpecimenBar);
        this.addAction(moveToDistanceThreshold);

        SpecimenHang specimenHang = new SpecimenHang(outtake);
        specimenHang.setDependentActions(specimenHangReady1, moveToDistanceThreshold);
        this.addAction(specimenHang);

    }
}
