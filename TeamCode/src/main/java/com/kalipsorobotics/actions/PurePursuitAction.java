package com.kalipsorobotics.actions;

import android.util.Log;

import com.kalipsorobotics.localization.SparkfunOdometry;
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
    SparkfunOdometry sparkfunOdometry;

    private final PidNav pidX;
    private final PidNav pidY;
    private final PidNav pidAngle;

    Path path;
    Segment lastLine;
    double radius = 1;
    Optional<Position> follow;
    Optional<Position> prevFollow;

    public PurePursuitAction(DriveTrain driveTrain, SparkfunOdometry sparkfunOdometry, WheelOdometry wheelOdometry) {
        this.driveTrain = driveTrain;
        this.sparkfunOdometry = sparkfunOdometry;

//        this.pidX = new PidNav(1. / 900, 0, 0);
//        this.pidY = new PidNav(1. / 900, 0, 0);
//        this.pidAngle = new PidNav(1 / 3.140, 0, 0);

        this.pidX = new PidNav(0.015, 0, 0);
        this.pidY = new PidNav(0.025, 0, 0);
        this.pidAngle = new PidNav(0.11, 0, 0);
        //0.001, 0.001, 0.2 behavior: turns slow and does slow glitches out
        //0.001, 0.001, 0.3 behavior: turns and then does not move
        //0.001, 0.001, 0.4 behavior: turns and then does not move
    //0.001, 0.001, 0.5 behavior: turns and then does not move
        //0.001, 0.001, 0.6 behavior: turns and then does not move
        //0.001, 0.001, 0.7 behavior: fast turn then does not move
        //0.001, 0.001, 0.8 behavior: fast turn then does not move
        //0.001, 0.001, 0.9 behavior: fast turn then does not move
        //0.001, 0.001, 1 behavior: fast turn then does not move

        //0.002, 0.002, 0.2 behavior: turns the right direction and then actually glitches tf out
        //0.003, 0.003, 0.2 behavior: turns a little in the right direction adn then goes wrong way
        //0.005, 0.005, 0.2 behavior: does not turn and goes the wrong way first

        Log.d("purepursaction", "constructed");

        this.dependentAction = new DoneStateAction();
    }

    public void addPoint(int x, int y, double heading) {
        pathPoints.add(new Position(x, y, heading));
        Log.d("purepursaction", "added point " + x + ", " + y);
    }

    private void targetPosition(Position target) {
        Position currentPos = sparkfunOdometry.getCurrentPosition();
        Vector currentToTarget = Vector.between(currentPos, target);

        double distanceToTarget = currentToTarget.getLength();
        double targetDirection = currentToTarget.getHeadingDirection();

        double targetAngle = target.getTheta();
//        if (distanceToTarget <= 100) {
//            targetAngle = preferredAngle;
//        } else {
//            targetAngle = currentToTarget.getHeadingDirection();
//        }
        Log.d("purepursaction", "target angle is " + targetAngle);

        double angleError = MathFunctions.angleWrapRad(targetAngle - currentPos.getTheta());
        double directionError = MathFunctions.angleWrapRad(targetDirection - currentPos.getTheta());
        Log.d("purepursaction", "angle error " + angleError);

//        opModeUtilities.getTelemetry().addData("targetAngle", targetAngle);
//        opModeUtilities.getTelemetry().addData("angle error", angleError);
//        opModeUtilities.getTelemetry().addData("direction error", directionError);

        double xError = Math.cos(directionError) * distanceToTarget;
        double powerX = pidX.getPower(xError);
        Log.d("purepursx", "set x " + powerX);

        double yError = Math.sin(directionError) * distanceToTarget;
        Log.d("purepursy", "yerror " + yError);
        double powerY = pidY.getPower(yError);
        Log.d("purepursy", "set y " + powerY);

        double powerAngle = pidAngle.getPower(angleError);
//        if(Math.abs(angleError) < 0.5) {
//            powerAngle = 0;
//        }

//        opModeUtilities.getTelemetry().addData("power angle", powerAngle);

        double fLeftPower = powerX + powerY + powerAngle;
        double bLeftPower = powerX - powerY + powerAngle;

        double fRightPower = powerX - powerY - powerAngle;
        double bRightPower = powerX + powerY - powerAngle;

        Log.d("purepursactionlog", "set power values " + fLeftPower + " " + fRightPower + " " + bLeftPower + " " + bRightPower);

        double biggestPower = MathFunctions.maxAbsValueDouble(fLeftPower, fRightPower, bLeftPower, bRightPower);
//        opModeUtilities.getTelemetry().addData("biggest power", biggestPower);
        if (biggestPower > 1) {
            fLeftPower /= biggestPower;
            fRightPower /= biggestPower;
            bLeftPower /= biggestPower;
            bRightPower /= biggestPower;
        }
        //92.66611361922297, -235.81741858351225

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

//        opModeUtilities.getTelemetry().addData("current pos", currentPos.toString());
//        opModeUtilities.getTelemetry().addData("power x ", powerX);
//        opModeUtilities.getTelemetry().addData("power y", powerY);
//        opModeUtilities.getTelemetry().addData("distance to target", distanceToTarget);
    }

    @Override
    public boolean checkDoneCondition() {

        lastLine = path.getSegment(path.numSegments() - 1);
        boolean distanceToEndWithinThreshold = sparkfunOdometry.getCurrentPosition().distanceTo(lastLine.getFinish()) < 1;
        //boolean velocityWithinThreshold = odometry.getCurrentVelocity().isWithinThreshhold(0.1, 0.1, Math.toRadians(5));
        boolean angleWithinRange = Math.abs(sparkfunOdometry.getCurrentPosition().getTheta() - pathPoints.get(path.numPoints()-1).getTheta()) < Math.toRadians(4);

        //Log.d("purepursaction_done", distanceToEndWithinThreshold + " " + velocityWithinThreshold + " " + angleWithinRange);

        if (!follow.isPresent()) {
            //opModeUtilities.getTelemetry().addLine("breake");
            //opModeUtilities.getTelemetry().update();
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
            //Log.d("purepursaction", "set path");
            hasStarted = true;
        }

        //Log.d("purepursaction", "entered update");

        //follow = path.searchFrom(odometry.getCurrentPosition(), radius);
        follow = path.lookAhead(sparkfunOdometry.getCurrentPosition(), radius);

        if (follow.isPresent()) {
            Log.d("purepursaction_debug_follow", follow.get().getX() + " " + follow.get().getY() + " " + follow.get().getTheta());
//                opModeUtilities.getTelemetry().addData("preferredAngle", preferredAngle);
//                opModeUtilities.getTelemetry().addData("follow point", follow);
            targetPosition(follow.get());
            //Log.d("position", odometry.getCurrentPosition().toString());
            //Log.d("velocity", odometry.getCurrentVelocity().toString());

            radius = 4; //50
        } else {
            driveTrain.setPower(0);
            Log.d("purepursaction_debug_follow", "done");
        }

        //if (Thread.interrupted()) throw new InterruptedException();

    }
}
