package com.kalipsorobotics.actions.drivetrain;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;

public class AngleLockTeleOp extends KActionSet {

    public AngleLockTeleOp(DriveTrain driveTrain, WheelOdometry wheelOdometry) {
        PurePursuitAction lockAngle = new PurePursuitAction(driveTrain, wheelOdometry);
        lockAngle.setName("lockAngle");
        lockAngle.addPoint(wheelOdometry.getCurrentPosition().getX(), wheelOdometry.getCurrentPosition().getY(), 0);
        this.addAction(lockAngle);
    }

}
