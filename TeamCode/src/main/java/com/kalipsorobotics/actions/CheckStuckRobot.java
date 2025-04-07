package com.kalipsorobotics.actions;
import static java.lang.Math.abs;

import android.app.SharedElementCallback;
import android.os.SystemClock;
import android.util.Log;

import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.math.Path;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.test.checkStuck.CheckXY;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.kalipsorobotics.utilities.SharedData;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

public class CheckStuckRobot {
    private double prevXPos = 0;
    private double prevYPos = 0;
    private double prevThetaPos = 0;
    private double prevXVelocity = 0;
    private double timeInMsSinceLastChecked = 0;
    private double timeOffset = 0;
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
    private final OpModeUtilities opModeUtilities;
    private PurePursuitAction purePursuitAction;
    private CheckXY checkXY;

    public CheckStuckRobot(DriveTrain driveTrain, WheelOdometry wheelOdometry, OpModeUtilities opModeUtilities, PurePursuitAction purePursuitAction){
        this.wheelOdometry = wheelOdometry;
        this.driveTrain = driveTrain;
        this.opModeUtilities = opModeUtilities;
        this.purePursuitAction = purePursuitAction;
        checkXY = new CheckXY(opModeUtilities);
    }

    private double getXDelta(Position currentPosition) {
        double currentxDelta = currentPosition.getX() - prevXPos;
        prevXPos = currentPosition.getX();
        return currentxDelta;
    }
    private double getYDelta(Position currentPosition) {
        double currentyDelta = currentPosition.getY() - prevYPos;
        prevYPos = currentPosition.getY();
        return currentyDelta;
    }
    private double getThetaDelta(Position currentPosition) {
        double currentThetaDelta = currentPosition.getTheta() - prevThetaPos;
        prevThetaPos = currentPosition.getTheta();
        return currentThetaDelta;
    }

//    private boolean isXDeltaValid() {
//        double currentXVelocity = wheelOdometry.getCurrentVelocity().getX();
//        double deltaXVelocity = abs(prevXVelocity - currentXVelocity);
//
//        if (deltaXVelocity < X_DELTA_MIN_THRESHOLD) {
//            return true;
//        }
//        return false;
//    }
//
//    private boolean isYDeltaValid() {
//        double currentYVelocity = wheelOdometry.getCurrentVelocity().getY();
//        double deltaYVelocity = abs(prevYVelocity - currentYVelocity);
//
//        if (deltaYVelocity < Y_DELTA_MIN_THRESHOLD) {
//            return true;
//        }
//        return false;
//    }
//
//    private boolean isThetaDeltaValid() {
//        double currentThetaVelocity = wheelOdometry.getCurrentVelocity().getTheta();
//        double deltaThetaVelocity = abs(prevThetaVelocity - currentThetaVelocity);
//
//        if (deltaThetaVelocity < THETA_DELTA_MIN_THRESHOLD) {
//            return true;
//        }
//        return false;
//    }
//    private boolean checkDeltaValid() {
//        if (isThetaDeltaValid() && isXDeltaValid() && isYDeltaValid()) {
//            return true;
//        }
//        return false;
//    }

    private boolean checkRobotNotMoving(double currentXVelocity, double currentYVelocity, double currentTimeInMs) {
        if (currentYVelocity < 0.05 || currentXVelocity < 0.05) {
            return true;
        }
        if (getXDelta(SharedData.getOdometryPosition()) < X_DELTA_MIN_THRESHOLD && getYDelta(SharedData.getOdometryPosition()) < Y_DELTA_MIN_THRESHOLD) {
            return true;
        }
        return false;
    }
    private boolean checkRobotSpinning(double currentXVelocity, double currentYVelocity, double currentThetaVelocity, Position currentPosition, double currentheading, double timeInMillis) {
        if (abs(currentThetaVelocity) < THETA_DELTA_MIN_THRESHOLD && (abs(currentXVelocity) < X_DELTA_MIN_THRESHOLD && abs(currentYVelocity) < Y_DELTA_MIN_THRESHOLD)) {
            double prevHeading = currentheading;
            currentheading = currentPosition.getTheta();
            // Check for consistent angular velocity
            if (Math.abs(currentheading - prevHeading) < 1) {
                return true;
            }
        }
        return false;
    }
    private boolean checkIfOnPath(/*Path path,*/ int timeInMillis) {
        //return checkXY.isPositionOnPath(path, timeInMillis);
        return true;
    }

    // change from out of void when method finished
    // if delta x, y, and theta are too low ( make threshold large ) then check the path and current pos
    public boolean isStuck(/*Path path,*/ int timeInMillis) {
        Position currentPos = SharedData.getOdometryPosition();
        Position intendedPos = currentPos;

        //TODO add intended pos
        double currentX = currentPos.getX();
        double currentY = currentPos.getY();
        double currentTheta = currentPos.getTheta();

//        double currentThetaVelocity = wheelOdometry.getCurrentVelocity().getTheta();
//        double deltaThetaVelocity = abs(prevThetaVelocity - currentThetaVelocity);
//
//        double currentYVelocity = wheelOdometry.getCurrentVelocity().getY();
//        double deltaYVelocity = abs(prevYVelocity - currentYVelocity);
//
//        double currentXVelocity = wheelOdometry.
//        double deltaXVelocity = abs(prevXVelocity - currentXVelocity);

        timeOffset = timeInMillis - timeInMsSinceLastChecked;
        timeInMsSinceLastChecked += timeInMillis - timeOffset;
        if (timeInMsSinceLastChecked > 1000) {
            timeOffset = 0;
            timeInMsSinceLastChecked = 0;
            if (checkRobotSpinning(getXDelta(currentPos), getYDelta(currentPos), getThetaDelta(currentPos), currentPos, currentPos.getTheta(), timeInMillis) ||
                    checkRobotNotMoving(getXDelta(currentPos), getYDelta(currentPos), timeInMillis)/* ||
                checkIfOnPath(path, timeInMillis)*/) {

                //unstuckRobot(driveTrain, /*path,*/ timeInMillis);
                Log.d("check stuck", "---ROBOT IS STUCK---");
                return true;

            }
            Log.d("check stuck", "---robot is not stuck---");
        }
        return false;

    }

    private void unstuckRobot(DriveTrain driveTrain, int timeInMillis){
        purePursuitAction = new PurePursuitAction(driveTrain, wheelOdometry);
        purePursuitAction.setMaxTimeOutMS(500);
        Position currentPos = new Position(wheelOdometry.countLeft(), wheelOdometry.countBack(), wheelOdometry.getCurrentImuHeading());
        //TODO replace 10 with something
        Position possiblePos1 = new Position(currentPos.getX() - 10, currentPos.getY(), currentPos.getTheta());
        Position possiblePos2 = new Position(currentPos.getX() + 10, currentPos.getY(), currentPos.getTheta());
        Position possiblePos3 = new Position(currentPos.getX(), currentPos.getY() - 10, currentPos.getTheta());
        Position possiblePos4 = new Position(currentPos.getX(), currentPos.getY() + 10, currentPos.getTheta());
        if (!checkIfOnPath(/*path*/timeInMillis)) {
            purePursuitAction.addPoint(possiblePos1.getX(), possiblePos1.getY(), possiblePos1.getTheta());
            purePursuitAction.addPoint(possiblePos2.getX(), possiblePos2.getY(), possiblePos2.getTheta());
            purePursuitAction.addPoint(possiblePos3.getX(), possiblePos3.getY(), possiblePos3.getTheta());
            purePursuitAction.addPoint(possiblePos4.getX(), possiblePos4.getY(), possiblePos4.getTheta());
            return;
        }
        return;
    }

}
