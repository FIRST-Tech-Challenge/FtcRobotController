package com.kalipsorobotics.actions.autoActions;

import android.util.Log;

import com.kalipsorobotics.PID.PidNav;
import com.kalipsorobotics.utilities.SharedData;
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

    public static final double P_XY = 1.0/600.0;
    public static final double P_ANGLE = 0.7 * 1.0 / Math.toRadians(90);
    public static final double P_XY_FAST = 1.0/100.0;
    public static final double P_ANGLE_FAST = 1.0/ Math.toRadians(30);
    public static final double P_XY_SLOW = 1.0/800.0;
    public static final double P_ANGLE_SLOW = 1.0/ Math.toRadians(130);

    public static final double P_XY_SUPER_FAST = 1.0/50.0;
    public static final double P_ANGLE_SUPER_FAST = 1.0/ Math.toRadians(15);

    private double lastSearchRadius = LAST_RADIUS_MM;

    List<Position> pathPoints = new ArrayList<Position>();
    DriveTrain driveTrain;
    WheelOdometry wheelOdometry;
    //SparkfunOdometry sparkfunOdometry;

    private final PidNav pidX;
    private final PidNav pidY;
    private PidNav pidAngle;

    int maxCheckDoneCounter = 1;
    int checkDoneCounter = 0;

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

    private Position currentPosition = new Position(SharedData.getOdometryPosition());
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
        this.pidX = new PidNav(P_XY, 0, 0);
        this.pidY = new PidNav(P_XY, 0, 0);
        this.pidAngle = new PidNav(P_ANGLE, 0, 0);



        this.timeoutTimer = new ElapsedTime();

        this.prevFollow = Optional.empty();

        Log.d("purepursaction", "constructed");

        this.dependentActions.add(new DoneStateAction());
    }

    public PurePursuitAction(DriveTrain driveTrain/*, SparkfunOdometry sparkfunOdometry*/, WheelOdometry wheelOdometry, double pidXY, double pidAngle) {
        this.driveTrain = driveTrain;
        //this.sparkfunOdometry = sparkfunOdometry;
        this.wheelOdometry = wheelOdometry;
        this.pidX = new PidNav(pidXY, 0, 0);
        this.pidY = new PidNav(pidXY, 0, 0);
        this.pidAngle = new PidNav(pidAngle, 0, 0);

        this.timeoutTimer = new ElapsedTime();

        this.prevFollow = Optional.empty();

        Log.d("purepursaction", "constructed");

        this.dependentActions.add(new DoneStateAction());
    }

    public void addPoint(double x, double y, double headingDeg) {
        double headingRad = Math.toRadians(headingDeg);
        double[] adaptiveP = calcAdaptiveP(x, y, headingRad);
        pathPoints.add(new Position(x, y, headingRad, adaptiveP[0], adaptiveP[1]));
        Log.d("purepursaction", "added point " + x + ", " + y);
    }

    public void addPoint(double x, double y, double headingDeg, double pXY, double pAngle) {
        pathPoints.add(new Position(x, y, Math.toRadians(headingDeg), pXY, pAngle));
        Log.d("purepursaction", "added point " + x + ", " + y);
    }

    public double[] calcAdaptiveP(double x, double y, double theta) {
        final double SLOW = 1.0/600;
        final double FAST = 1.0/120;
        final double MAX = 1.0/70;
        final double ANGLE_THRESHOLD = Math.toRadians(45);
        final double MAX_ANGLE_THRESHOLD = Math.toRadians(180);

        Position pos = pathPoints.isEmpty() ? new Position(0, 0, 0) : pathPoints.get(pathPoints.size() - 1);
        double actionDistance = pos.distanceTo(new Position(x, y, theta));
        double headingDelta = Math.abs(pos.getTheta() - theta);

        double adaptiveXY;
        if (actionDistance < 100) {
            adaptiveXY = SLOW;
        } else if (actionDistance < 300) {
            adaptiveXY = SLOW + (actionDistance - 100) * (FAST - SLOW) / 200;
        } else if (actionDistance < 1000) {
            adaptiveXY = FAST + (actionDistance - 300) * (MAX - FAST) / 700;
        } else {
            adaptiveXY = FAST;
        }

        double adaptiveTheta = P_ANGLE;
        if (headingDelta > ANGLE_THRESHOLD) {
            adaptiveTheta += (headingDelta - ANGLE_THRESHOLD) / (MAX_ANGLE_THRESHOLD - ANGLE_THRESHOLD) * (1 - P_ANGLE);
        }

        Log.d("purepursaction_adaptiveP", String.format("action distance %f, P increased from %f to %f", actionDistance, P_XY, adaptiveXY));
        Log.d("purepursaction_adaptiveP", String.format("heading delta %f, P increased from %f to %f", headingDelta, P_ANGLE, adaptiveTheta));
        return new double[]{adaptiveXY, adaptiveTheta};
    }

    public void setFinalSearchRadius(double searchRadiusMM){
        this.lastSearchRadius = searchRadiusMM;
    }
    public void setPAngle(double p) {
        this.pidAngle = new PidNav(p, 0,0);
    }

    private void targetPosition(Position target, Position currentPos) {
        //Position currentPos = wheelOdometry.getCurrentPosition();
        Vector currentToTarget = Vector.between(currentPos, target);

        double distanceToTarget = currentToTarget.getLength();
        double targetDirection = currentToTarget.getHeadingDirection();
        double targetAngle = target.getTheta();
        double directionError = MathFunctions.angleWrapRad(targetDirection - currentPos.getTheta());

        double angleError = MathFunctions.angleWrapRad(targetAngle - currentPos.getTheta());
        double xError = Math.cos(directionError) * distanceToTarget;
        double yError = Math.sin(directionError) * distanceToTarget;

        double powerAngle = target.getPidAngle().getPower(angleError);
        double powerX = target.getPidX().getPower(xError);
        double powerY = target.getPidY().getPower(yError);

        Log.d("directionalpower", String.format("power x=%.4f, power y=%.5f, powertheta=%.6f", powerX, powerY,
                powerAngle));

        double fLeftPower = powerX + powerY + powerAngle;
        double bLeftPower = powerX - powerY + powerAngle;
        double fRightPower = powerX - powerY - powerAngle;
        double bRightPower = powerX + powerY - powerAngle;

        Log.d("PurePursuit_Log", "running " + name + "set power values " + fLeftPower + " " + fRightPower + " " + bLeftPower + " " + bRightPower);

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
        public void setIsDone(boolean isDone) {
            this.isDone = isDone;
            if (isDone) {
                driveTrain.setPower(0);
            }
        }
    @Override
    public void update() {
        if (isDone) {
            return;
        }

        if (!hasStarted) {
            path = new Path(pathPoints);
            startTimeMS = System.currentTimeMillis();
            hasStarted = true;
            //lastPosition = wheelOdometry.getCurrentPosition();
            lastPosition = new Position(SharedData.getOdometryPosition());
            timeoutTimer.reset();
        }

        currentPosition = new Position(SharedData.getOdometryPosition());

        double elapsedTime = System.currentTimeMillis() - startTimeMS;

        if (elapsedTime >= maxTimeOutMS) {
            setIsDone(true);
            Log.d("purepursaction_debug_follow", "done timeout  " + getName());
            return;
        }


        currentLookAheadRadius = LOOK_AHEAD_RADIUS_MM;

        Position lastPoint = path.getLastPoint();

        if (prevFollow.isPresent() && (path.findIndex(prevFollow.get()) > (path.numPoints() - 2))) {
            currentLookAheadRadius = lastSearchRadius;
        }
        follow = path.lookAhead(currentPosition, prevFollow, currentLookAheadRadius);

        if (follow.isPresent()) {
            Log.d("purepursaction_debug_follow",
                    "follow point:  " + follow.get());
            Log.d("purepursaction_debug_follow", "current pos:    " + currentPosition.toString());
            targetPosition(follow.get(), currentPosition);

        } else {
            if (Math.abs(lastPoint.getTheta() - currentPosition.getTheta()) <= Math.toRadians(1.5) ) {
                finishedMoving();
            } else {
                Log.d("purepursaction_debug_follow",
                        "locking final angle:  " + lastPoint);
                Log.d("purepursaction_debug_follow", "current pos:    " + currentPosition.toString());
                targetPosition(lastPoint, currentPosition);
            }
        }

        xVelocity = (Math.abs(lastPosition.getX() - currentPosition.getX())) / (Math.abs(lastMilli - timeoutTimer.milliseconds()));
        yVelocity = (Math.abs(lastPosition.getY() - currentPosition.getY())) / (Math.abs(lastMilli - timeoutTimer.milliseconds()));
        thetaVelocity = (Math.abs(lastPosition.getTheta() - currentPosition.getTheta())) / (Math.abs(lastMilli - timeoutTimer.milliseconds()));

        if(xVelocity < 0.01 && yVelocity < 0.01 && thetaVelocity < 0.01) {
            if(timeoutTimer.milliseconds() > 1000) {
                finishedMoving();
            }
        } else {
            timeoutTimer.reset();
        }

        lastMilli = timeoutTimer.milliseconds();
        lastPosition = currentPosition;

    }


    public void setMaxCheckDoneCounter (int maxCheckDoneCounter) {
        this.maxCheckDoneCounter = maxCheckDoneCounter;
    }

    public void finishedMoving() {
        driveTrain.setPower(0);
        Log.d("purepursaction_debug_follow", "timeout");
        Log.d("purepursaction_debug_follow", "current pos:    " + currentPosition.toString());
        checkDoneCounter++;

        Log.d("purepursaction_debug_checkDone", "checkDoneCounter: " + checkDoneCounter + "name:" +  name);

        if (checkDoneCounter >= maxCheckDoneCounter) {
            Log.d("purepursaction_debug_checkDone", "Done" + checkDoneCounter + "name:" +  name + "Pos: " + currentPosition);
            isDone = true;
        }

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

    public WheelOdometry getOdometry() {
        return wheelOdometry;
    }
}
