package com.kalipsorobotics.actions.autoActions;

import static com.kalipsorobotics.actions.autoActions.WallToBarHangRoundTrip.WALL_PICKUP_X;
import static com.kalipsorobotics.actions.autoActions.WallToBarHangRoundTrip.WALL_PICKUP_Y;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.MoveToDistanceThreshold;
import com.kalipsorobotics.actions.outtake.DistanceDetectionAction;
import com.kalipsorobotics.actions.outtake.SpecimenWallReady;
import com.kalipsorobotics.actions.outtake.WallPickupDistanceSensorAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.Outtake;

public class BarToWallMoveReady extends KActionSet {

    public BarToWallMoveReady(DriveTrain driveTrain, WheelOdometry wheelOdometry, Outtake outtake) {
        SpecimenWallReady specimenWallReady = new SpecimenWallReady(outtake);
        specimenWallReady.setName("specimenWallReady");
        this.addAction(specimenWallReady);

        PurePursuitAction moveBarToWall = new PurePursuitAction(driveTrain, wheelOdometry); //300
        moveBarToWall.setName("moveBarToWall");
        this.addAction(moveBarToWall);
        moveBarToWall.setMaxTimeOutMS(3500);
        moveBarToWall.addPoint(-400, WALL_PICKUP_Y, -180, PurePursuitAction.P_XY_FAST,
                PurePursuitAction.P_ANGLE_FAST); //-205, 700
        moveBarToWall.addPoint(WALL_PICKUP_X + 25, WALL_PICKUP_Y, -180, PurePursuitAction.P_XY_SLOW,
                PurePursuitAction.P_ANGLE_SLOW);


        WallPickupDistanceSensorAction wallPickupDistanceSensorAction = new WallPickupDistanceSensorAction(outtake,
                moveBarToWall, driveTrain);
        wallPickupDistanceSensorAction.setName("wallPickupDistanceSensorAction");
        this.addAction(wallPickupDistanceSensorAction);
        wallPickupDistanceSensorAction.setDependentActions(specimenWallReady);


    }

}
