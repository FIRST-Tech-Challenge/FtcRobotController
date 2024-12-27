package com.kalipsorobotics.actions.autoActions;

import android.os.SystemClock;
import android.util.Log;

import com.kalipsorobotics.PID.PidNav;
import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.DoneStateAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.math.MathFunctions;
import com.kalipsorobotics.math.Path;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.math.Segment;
import com.kalipsorobotics.math.Vector;

import com.kalipsorobotics.modules.DriveTrain;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class PurePursuitAction extends Action {

    List<Position> pathPoints = new ArrayList<Position>();
    DriveTrain driveTrain;
    WheelOdometry wheelOdometry;
    //SparkfunOdometry sparkfunOdometry;

    private final PidNav pidX;
    private final PidNav pidY;
    private final PidNav pidAngle;

    Path path;
    Segment lastLine;
    static final private double LOOK_AHEAD_RADIUS_MM = 75;

    private double currentLookAheadRadius;
    static final private double LAST_RADIUS_MM = 15;
    Optional<Position> follow;
    Optional<Position> prevFollow;

    private long sleepTimeMS = 0;

    private double maxTimeOutMS = 1000000000;

    private double startTimeMS = System.currentTimeMillis();

    private Position lastPosition;
    private double lastMilli = 0;
    ElapsedTime timeoutTimer;

    private double xVelocity;
    private double yVelocity;
    private double thetaVelocity;
//    private final double threshold = 10;

    /**
     * Should not do more than 24 inches or 600mm moves in X and Y (single move)
     * Should not turn more than 90 deg (single move)
     * If move more than normal range --> add more waypoints
     */

    public PurePursuitAction(DriveTrain driveTrain/*, SparkfunOdometry sparkfunOdometry*/,
                             WheelOdometry wheelOdometry) {
        this.driveTrain = driveTrain;
        //this.sparkfunOdometry = sparkfunOdometry;
        this.wheelOdometry = wheelOdometry;
        this.pidX = new PidNav(1.0/600.0, 0, 0);
        this.pidY = new PidNav(1.0/600.0, 0, 0);
        this.pidAngle = new PidNav(0.7 * (1.0 / Math.toRadians(90)), 0, 0);

        this.timeoutTimer = new ElapsedTime();

        this.prevFollow = Optional.empty();

        Log.d("purepursaction", "constructed");

        this.dependentActions.add(new DoneStateAction());
    }

    public void addPoint(double x, double y, double headingDeg) {
        pathPoints.add(new Position(x, y, Math.toRadians(headingDeg)));
        Log.d("purepursaction", "added point " + x + ", " + y);
    }

    private void targetPosition(Position target) {
        Position currentPos = wheelOdometry.getCurrentPosition();
        Vector currentToTarget = Vector.between(currentPos, target);

        double distanceToTarget = currentToTarget.getLength();
        double targetDirection = currentToTarget.getHeadingDirection();
        double targetAngle = target.getTheta();
        double directionError = MathFunctions.angleWrapRad(targetDirection - currentPos.getTheta());

        double angleError = MathFunctions.angleWrapRad(targetAngle - currentPos.getTheta());
        double xError = Math.cos(directionError) * distanceToTarget;
        double yError = Math.sin(directionError) * distanceToTarget;

        double powerAngle = pidAngle.getPower(angleError);
        double powerX = pidX.getPower(xError);
        double powerY = pidY.getPower(yError);

        Log.d("directionalpower", String.format("power x=%.4f, power y=%.5f, powertheta=%.6f", powerX, powerY,
                powerAngle));

        double fLeftPower = powerX + powerY + powerAngle;
        double bLeftPower = powerX - powerY + powerAngle;

        double fRightPower = powerX - powerY - powerAngle;
        double bRightPower = powerX + powerY - powerAngle;

        Log.d("purepursactionlog", "set power values " + fLeftPower + " " + fRightPower + " " + bLeftPower + " " + bRightPower);

        driveTrain.setPowerWithRangeClippingMinThreshold(fLeftPower, fRightPower, bLeftPower, bRightPower, 0.4);
//        driveTrain.setPower(fLeftPower, fRightPower, bLeftPower, bRightPower);
        Log.d("purepursactionlog", "target position " + target.getX() + " " + target.getY() + " " + targetAngle);
        prevFollow = Optional.of(target);
    }

    @Override
    public boolean checkDoneCondition() {
        return isDone;
    }

    @Override
    public void update() {
        super.update();
        if (isDone) {
            return;
        }

        if (!hasStarted) {
            path = new Path(pathPoints);
            startTimeMS = System.currentTimeMillis();
            hasStarted = true;
            lastPosition = wheelOdometry.getCurrentPosition();
            timeoutTimer.reset();
        }


        double elapsedTime = System.currentTimeMillis() - startTimeMS;

        if (elapsedTime >= maxTimeOutMS) {
            setIsDone(true);
            Log.d("purepursaction_debug_follow", "done timeout  " + getName());
            return;
        }


        currentLookAheadRadius = LOOK_AHEAD_RADIUS_MM;

        Position lastPoint = path.getLastPoint();
//        if (lastPoint.distanceTo(wheelOdometry.getCurrentPosition()) > threshold) {  // for small distances
//            currentLookAheadRadius = LAST_RADIUS_MM;
//        }

        if (prevFollow.isPresent() && (path.findIndex(prevFollow.get()) > (path.numPoints() - 2))) {
            currentLookAheadRadius = LAST_RADIUS_MM;
        }
        follow = path.lookAhead(wheelOdometry.getCurrentPosition(), currentLookAheadRadius);

        if (follow.isPresent()) {
            Log.d("purepursaction_debug_follow",
                    "follow point:  " + follow.get());
            Log.d("purepursaction_debug_follow", "current pos:    " + wheelOdometry.getCurrentPosition().toString());
            targetPosition(follow.get());

        } else {
            if (Math.abs(lastPoint.getTheta() - wheelOdometry.getCurrentPosition().getTheta()) <= Math.toRadians(2) ) {
                finishedMoving();
            } else {
                Log.d("purepursaction_debug_follow",
                        "locking final angle:  " + lastPoint);
                Log.d("purepursaction_debug_follow", "current pos:    " + wheelOdometry.getCurrentPosition().toString());
                targetPosition(lastPoint);
            }
        }

        xVelocity = (Math.abs(lastPosition.getX() - wheelOdometry.getCurrentPosition().getX())) / (Math.abs(lastMilli - timeoutTimer.milliseconds()));
        yVelocity = (Math.abs(lastPosition.getY() - wheelOdometry.getCurrentPosition().getY())) / (Math.abs(lastMilli - timeoutTimer.milliseconds()));
        thetaVelocity = (Math.abs(lastPosition.getTheta() - wheelOdometry.getCurrentPosition().getTheta())) / (Math.abs(lastMilli - timeoutTimer.milliseconds()));

        if(xVelocity < 0.0075 && yVelocity < 0.0075 && thetaVelocity < 0.0075) {
            if(timeoutTimer.milliseconds() > 2000) {
                finishedMoving();
            }
        } else {
            timeoutTimer.reset();
        }

        lastMilli = timeoutTimer.milliseconds();
        lastPosition = wheelOdometry.getCurrentPosition();

    }

    public void finishedMoving() {
        driveTrain.setPower(0);
        Log.d("purepursaction_debug_follow", "timeout");
        Log.d("purepursaction_debug_follow", "current pos:    " + wheelOdometry.getCurrentPosition().toString());
        isDone = true;
        if (sleepTimeMS != 0) {
            try {
                Thread.sleep(sleepTimeMS);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    public void setSleep(long sleepTimeMS) {
        this.sleepTimeMS = sleepTimeMS;
    }

    public double getMaxTimeOutMS() {
        return maxTimeOutMS;
    }

    public void setMaxTimeOutMS(double maxTimeOutMS) {
        this.maxTimeOutMS = maxTimeOutMS;
    }
}
