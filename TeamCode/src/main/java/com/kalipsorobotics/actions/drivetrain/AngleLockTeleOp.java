package com.kalipsorobotics.actions.drivetrain;

import com.kalipsorobotics.utilities.SharedData;
import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;

public class AngleLockTeleOp extends KActionSet {

    public AngleLockTeleOp(DriveTrain driveTrain, WheelOdometry wheelOdometry) {
        PurePursuitAction lockAngle = new PurePursuitAction(driveTrain, wheelOdometry);
        lockAngle.setName("lockAngle");
        lockAngle.addPoint(SharedData.getOdometryPosition().getX(), SharedData.getOdometryPosition().getY(), 0.1);
        this.addAction(lockAngle);
    }

}
