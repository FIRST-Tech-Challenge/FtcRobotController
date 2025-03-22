package com.kalipsorobotics.actions;

import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.modules.DriveTrain;

public class CheckStuckRobot {
    public CheckStuckRobot(DriveTrain driveTrain){
        /*
        public boolean isStuck() {
            double deltaXVelocity = Math.abs(prevXVelocity-xVelocity);
            double deltaYVelocity = Math.abs(prevYVelocity-yVelocity);

            boolean littleVelocityChange = deltaYVelocity < STUCK_THRESHOLD && deltaXVelocity < STUCK_THRESHOLD;

            prevXVelocity = xVelocity;
            prevYVelocity = yVelocity;

            if (littleVelocityChange && timeoutTimer.milliseconds() > 1000) {
                unstuckRobot();
                return true;
            } else {
                timeoutTimer.reset();
                return false;
            }
        }

        private void unstuckRobot(){

        }

         */


    }
}
