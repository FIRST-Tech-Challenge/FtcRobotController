package com.kalipsorobotics.actions.drivetrain;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;

public class MoveWallTeleOp extends KActionSet {

    public MoveWallTeleOp(DriveTrain driveTrain, WheelOdometry wheelOdometry) {
        PurePursuitAction moveToWall = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToWall.setName("moveToWall");
        moveToWall.addPoint(-150, 0, -180);
        moveToWall.addPoint(0, 0, -180);
        this.addAction(moveToWall);
    }

}
