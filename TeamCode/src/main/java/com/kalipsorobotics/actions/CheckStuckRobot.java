package com.kalipsorobotics.actions;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.utilities.SharedData;

public class CheckStuckRobot {
    private double prevXVelocity = 0;
    /**
     * mm per second
     * */
    final private double X_DELTA_MIN_THRESHOLD = 0.1; // to be calc
    private double prevYVelocity = 0;
    final private double Y_DELTA_MIN_THRESHOLD = 0.1; // to be calc

    private double prevThetaVelocity = 0;

    final private double THETA_DELTA_MIN_THRESHOLD = 0.1; // to be calc

    private final WheelOdometry wheelOdometry;
    private final DriveTrain driveTrain;
    private PurePursuitAction purePursuitAction;

    public CheckStuckRobot(DriveTrain driveTrain, WheelOdometry wheelOdometry, PurePursuitAction purePursuitAction){
        this.wheelOdometry = wheelOdometry;
        this.driveTrain = driveTrain;
        this.purePursuitAction = purePursuitAction;
    }

    private boolean isXDeltaValid() {
        double currentXVelocity = wheelOdometry.getCurrentVelocity().getX();
        double deltaXVelocity = Math.abs(prevXVelocity - currentXVelocity);

        if (deltaXVelocity < X_DELTA_MIN_THRESHOLD) {
            return true;
        }
        return false;
    }

    private boolean isYDeltaValid() {
        double currentYVelocity = wheelOdometry.getCurrentVelocity().getY();
        double deltaYVelocity = Math.abs(prevYVelocity - currentYVelocity);

        if (deltaYVelocity < Y_DELTA_MIN_THRESHOLD) {
            return true;
        }
        return false;
    }

    private boolean isThetaDeltaValid() {
        double currentThetaVelocity = wheelOdometry.getCurrentVelocity().getTheta();
        double deltaThetaVelocity = Math.abs(prevThetaVelocity - currentThetaVelocity);

        if (deltaThetaVelocity < THETA_DELTA_MIN_THRESHOLD) {
            return true;
        }
        return false;
    }

    // change from out of void when method finished
    // if delta x, y, and theta are too low ( make threshold large ) then check the path and current pos
    public void isStuck() {
        Position currentPos = SharedData.getOdometryPosition();
        double currentX = currentPos.getX();
        double currentY = currentPos.getY();
        double currentTheta = currentPos.getTheta();



    }


    private void unstuckRobot(DriveTrain driveTrain){
        PurePursuitAction test = new PurePursuitAction(driveTrain, wheelOdometry);

    }

}
