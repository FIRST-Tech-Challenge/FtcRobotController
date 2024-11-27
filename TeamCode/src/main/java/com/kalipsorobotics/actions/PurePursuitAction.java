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
    static final private double LOOK_AHEAD_RADIUS_MM = 15;
    Optional<Position> follow;
    Optional<Position> prevFollow;

    public PurePursuitAction(DriveTrain driveTrain/*, SparkfunOdometry sparkfunOdometry*/,
                             WheelOdometry wheelOdometry) {
        this.driveTrain = driveTrain;
        //this.sparkfunOdometry = sparkfunOdometry;
        this.wheelOdometry = wheelOdometry;
        this.pidX = new PidNav(0.015, 0, 0);
        this.pidY = new PidNav(0.025, 0, 0);
        this.pidAngle = new PidNav(0.30, 0, 0);


        Log.d("purepursaction", "constructed");

        this.dependentAction = new DoneStateAction();
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

        Log.d("purepursaction", "target angle is " + targetAngle);

        double angleError = MathFunctions.angleWrapRad(targetAngle - currentPos.getTheta());
        double directionError = MathFunctions.angleWrapRad(targetDirection - currentPos.getTheta());
        Log.d("purepursaction", "angle error " + angleError);

        double xError = Math.cos(directionError) * distanceToTarget;
        double powerX = pidX.getPower(xError);
        Log.d("purepursx", "set x " + powerX);

        double yError = Math.sin(directionError) * distanceToTarget;
        Log.d("purepursy", "yerror " + yError);

        double powerY = pidY.getPower(yError);
        Log.d("purepursy", "set y " + powerY);

        double powerAngle = pidAngle.getPower(angleError);
        Log.d("powerJimmeh", String.format("powerX = %.4f, powerY = %.4f, powerAngle = %.4f", powerX, powerY,
                powerAngle));
        double fLeftPower = powerX + powerY + powerAngle;
        double bLeftPower = powerX - powerY + powerAngle;

        double fRightPower = powerX - powerY - powerAngle;
        double bRightPower = powerX + powerY - powerAngle;

        Log.d("purepursactionlog", "set power values " + fLeftPower + " " + fRightPower + " " + bLeftPower + " " + bRightPower);

        double biggestPower = MathFunctions.maxAbsValueDouble(fLeftPower, fRightPower, bLeftPower, bRightPower);
        if (biggestPower > 1) {
            fLeftPower /= biggestPower;
            fRightPower /= biggestPower;
            bLeftPower /= biggestPower;
            bRightPower /= biggestPower;
        }

        if(Math.abs(fLeftPower) < 0.15) {
            if (fLeftPower < 0) {
                fLeftPower += -0.05;
            } else {
                fLeftPower += 0.05;
            }
        }
        if(Math.abs(fRightPower) < 0.15) {
            if (fRightPower < 0) {
                fRightPower += -0.05;
            } else {
                fRightPower += 0.05;
            }
        }
        if(Math.abs(bLeftPower) < 0.15) {
            if (bLeftPower < 0) {
                bLeftPower += -0.05;
            } else {
                bLeftPower += 0.05;
            }
        }
        if(Math.abs(bRightPower) < 0.15) {
            if (bRightPower < 0) {
                bRightPower += -0.05;
            } else {
                bRightPower += 0.05;
            }
        }

        Log.d("purepursactionlog", "target position " + target.getX() + " " + target.getY() + " " + targetAngle);
        driveTrain.setPower(fLeftPower, fRightPower, bLeftPower, bRightPower);
        prevFollow = Optional.of(target);
    }

    @Override
    public boolean checkDoneCondition() {

        lastLine = path.getSegment(path.numSegments() - 1);
        boolean distanceToEndWithinThreshold = wheelOdometry.getCurrentPosition().distanceTo(lastLine.getFinish()) < 1;
        boolean angleWithinRange =
                Math.abs(wheelOdometry.getCurrentPosition().getTheta() - pathPoints.get(path.numPoints()-1).getTheta()) < Math.toRadians(4);


        if (!follow.isPresent()) {
            driveTrain.setPower(0, 0, 0, 0);
            Log.d("purepursaction_done", "done");
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void update() {
        if(!hasStarted) {
            path = new Path(pathPoints);
            hasStarted = true;
        }


        follow = path.lookAhead(wheelOdometry.getCurrentPosition(), LOOK_AHEAD_RADIUS_MM);

        if (follow.isPresent()) {
            Log.d("purepursaction_debug_follow",
                    "follow point:  " + follow.get().toString());
            Log.d("purepursaction_debug_follow", "current pos:    " + wheelOdometry.getCurrentPosition().toString());
            targetPosition(follow.get());

        } else {

            if (Math.abs(prevFollow.get().getTheta() - wheelOdometry.getCurrentPosition().getTheta()) <= Math.toRadians(2)) {
                driveTrain.setPower(0);
            } else {
                incrementAnglePID(0, 0.000001, 0);
                targetPosition(prevFollow.get());
            }

            Log.d("purepursaction_debug_follow", "done");

        }
    }

    public void incrementAnglePID(double deltaKP, double deltaKI, double deltaKD) {
        pidAngle.setP(pidAngle.getP()+deltaKP);
        pidAngle.setI(pidAngle.getI()+deltaKI);
        pidAngle.setD(pidAngle.getD()+deltaKD);
        Log.d("PID_angle_2", pidAngle.toString());
    }

    public void incrementXPID(double deltaKP, double deltaKI, double deltaKD) {
        pidX.setP(pidX.getP()+deltaKP);
        pidX.setI(pidX.getI()+deltaKI);
        pidX.setD(pidX.getD()+deltaKD);
    }

    public void incrementYPID(double deltaKP, double deltaKI, double deltaKD) {
        pidY.setP(pidY.getP()+deltaKP);
        pidY.setI(pidY.getI()+deltaKI);
        pidY.setD(pidY.getD()+deltaKD);
    }

    public PidNav getPidAngle() {
        return pidAngle;
    }
}
