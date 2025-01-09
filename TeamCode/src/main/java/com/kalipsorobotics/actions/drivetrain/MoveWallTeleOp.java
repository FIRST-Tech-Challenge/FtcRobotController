package com.kalipsorobotics.actions.drivetrain;

import static com.kalipsorobotics.actions.autoActions.WallToBarHangRoundTrip.WALL_PICKUP_PID_VALUE;
import static com.kalipsorobotics.actions.autoActions.WallToBarHangRoundTrip.WALL_PICKUP_X;
import static com.kalipsorobotics.actions.autoActions.WallToBarHangRoundTrip.WALL_PICKUP_Y;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.actions.autoActions.WallToBarHangRoundTrip;
import com.kalipsorobotics.intoTheDeep.AutoSpecimen;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;

public class MoveWallTeleOp extends KActionSet {

    public MoveWallTeleOp(DriveTrain driveTrain, WheelOdometry wheelOdometry) {
        PurePursuitAction moveBarToWall = new PurePursuitAction(driveTrain, wheelOdometry,1.0/100.0);
        moveBarToWall.setName("moveBarToWall");
        moveBarToWall.setMaxTimeOutMS(3500);
        moveBarToWall.addPoint(WALL_PICKUP_X-100, WALL_PICKUP_Y, -180); //-205, 700
        this.addAction(moveBarToWall);
//
//        PurePursuitAction moveToDepot = new PurePursuitAction(driveTrain,wheelOdometry, WALL_PICKUP_PID_VALUE);
//        moveToDepot.setName("moveToDepot");
//        moveToDepot.setDependentActions(moveBarToWall);
//        //to depot for specimen
//        moveToDepot.setMaxTimeOutMS(3000);
//        moveToDepot.addPoint(WALL_PICKUP_X, WALL_PICKUP_Y, -180); //-130, -615
//        this.addAction(moveToDepot);

    }

}
