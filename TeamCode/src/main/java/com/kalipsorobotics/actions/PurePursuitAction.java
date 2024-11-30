package com.kalipsorobotics.actions;

import android.util.Log;

import com.kalipsorobotics.PID.PidNav;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.math.MathFunctions;
import com.kalipsorobotics.math.Path;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.math.Segment;
import com.kalipsorobotics.math.Vector;

import com.kalipsorobotics.modules.DriveTrain;

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

        this.prevFollow = Optional.empty();

        Log.d("purepursaction", "constructed");

        this.dependentActions.add(new DoneStateAction());
    }

    public void addPoint(int x, int y, double headingDeg) {
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
        double powerAngle = pidAngle.getPower(angleError);


        double xError = Math.cos(directionError) * distanceToTarget;
        double powerX = pidX.getPower(xError);

        double yError = Math.sin(directionError) * distanceToTarget;
        double powerY = pidY.getPower(yError);

        Log.d("directionalpower", String.format("power x=%.4f, power y=%.5f, powertheta=%.6f", powerX, powerY,
                powerAngle));
        double fLeftPower = powerX + powerY + powerAngle;
        double bLeftPower = powerX - powerY + powerAngle;

        double fRightPower = powerX - powerY - powerAngle;
        double bRightPower = powerX + powerY - powerAngle;

        Log.d("purepursactionlog", "set power values " + fLeftPower + " " + fRightPower + " " + bLeftPower + " " + bRightPower);

        driveTrain.setPowerWithRangeClippingMinThreshold(fLeftPower, fRightPower, bLeftPower, bRightPower, 0.4);

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
            hasStarted = true;
        }

        currentLookAheadRadius = LOOK_AHEAD_RADIUS_MM;

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
            Position lastPoint = path.getLastPoint();

            if ((lastPoint.getTheta() - wheelOdometry.getCurrentPosition().getTheta()) <= Math.toRadians(2) ) {
                driveTrain.setPower(0);
                Log.d("purepursaction_debug_follow", "done");
                isDone = true;
                if (sleepTimeMS != 0) {
                    try {
                        Thread.sleep(sleepTimeMS);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }
            } else {
                Log.d("purepursaction_debug_follow",
                        "locking final angle:  " + lastPoint);
                Log.d("purepursaction_debug_follow", "current pos:    " + wheelOdometry.getCurrentPosition().toString());
                targetPosition(lastPoint);
            }
        }
    }

    public void setSleep(long sleepTimeMS) {
        this.sleepTimeMS = sleepTimeMS;
    }

}
