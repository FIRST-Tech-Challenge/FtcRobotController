package com.kalipsorobotics.actions.drivetrain;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.actions.autoActions.WallToBarHangRoundTrip;
import com.kalipsorobotics.intoTheDeep.AutoSpecimen;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;

public class MoveWallTeleOp extends KActionSet {

    public MoveWallTeleOp(DriveTrain driveTrain, WheelOdometry wheelOdometry) {
        PurePursuitAction moveToWall = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToWall.setName("moveToWall");
        moveToWall.addPoint(-205, WallToBarHangRoundTrip.WALL_PICKUP_Y, -180);
        moveToWall.addPoint(WallToBarHangRoundTrip.WALL_PICKUP_X, WallToBarHangRoundTrip.WALL_PICKUP_Y, -180);
        this.addAction(moveToWall);
    }

}
