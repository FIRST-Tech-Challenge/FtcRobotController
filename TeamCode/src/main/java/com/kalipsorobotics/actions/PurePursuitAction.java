package com.kalipsorobotics.actions;

import android.util.Log;

import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.PID.PidNav;
import com.kalipsorobotics.math.MathFunctions;
import com.kalipsorobotics.math.Path;
import com.kalipsorobotics.math.Point;
import com.kalipsorobotics.math.Position;
import com.kalipsorobotics.math.Segment;
import com.kalipsorobotics.math.Vector;

import com.qualcomm.robotcore.util.Range;

import com.kalipsorobotics.modules.DriveTrain;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class PurePursuitAction extends Action {

    List<Point> pathPoints = new ArrayList<Point>();
    DriveTrain driveTrain;
    Odometry odometry;

    private final PidNav pidX;
    private final PidNav pidY;
    private final PidNav pidAngle;

    Path path;
    Segment lastLine;
    double preferredAngle;
    double radius;
    Optional<Point> follow;

    public PurePursuitAction(DriveTrain driveTrain, Odometry odometry) {
        this.driveTrain = driveTrain;
        this.odometry = odometry;

        this.pidX = new PidNav(1. / 900, 0, 0);
        this.pidY = new PidNav(1. / 900, 0, 0);
        this.pidAngle = new PidNav(1 / 3.140, 0, 0);
        Log.d("purepursaction", "constructed");

        this.dependentAction = new DoneStateAction();
    }

    public void addPoint(int x, int y) {
        pathPoints.add(new Point(x, y));
        Log.d("purepursaction", "added point " + x + ", " + y);
    }

    private void targetPosition(Point target, double preferredAngle) {
        Position currentPos = odometry.getCurrentPosition();
        Vector currentToTarget = Vector.between(currentPos.toPoint(), target);

        double distanceToTarget = currentToTarget.getLength();
        double targetDirection = currentToTarget.getHeadingDirection();

        double targetAngle;
        if (distanceToTarget <= 100) {
            targetAngle = preferredAngle;
        } else {
            targetAngle = -currentToTarget.getHeadingDirection();
            Log.d("purepursaction", "target angle is " + targetAngle);
        }

        double angleError = MathFunctions.angleWrapRad(targetAngle - currentPos.getTheta());
        double directionError = MathFunctions.angleWrapRad(targetDirection - currentPos.getTheta());
        Log.d("purepursaction", "angle error " + angleError);

//        opModeUtilities.getTelemetry().addData("targetAngle", targetAngle);
//        opModeUtilities.getTelemetry().addData("angle error", angleError);
//        opModeUtilities.getTelemetry().addData("direction error", directionError);

        double xError = Math.cos(directionError) * distanceToTarget;
        double powerX = Range.clip(pidX.getPower(xError), -1, 1);
        Log.d("purepursx", "set x " + powerX);

        double yError = Math.sin(directionError) * distanceToTarget;
        Log.d("purepursy", "yerror " + yError);
        double powerY = Range.clip(pidY.getPower(yError), -1, 1);
        Log.d("purepursy", "set y " + powerY);

        double powerAngle = Range.clip(pidAngle.getPower(angleError), -1, 1);
//        if(Math.abs(angleError) < 0.5) {
//            powerAngle = 0;
//        }

//        opModeUtilities.getTelemetry().addData("power angle", powerAngle);

        double fLeftPower = powerX - powerY + powerAngle;
        double fRightPower = powerX + powerY - powerAngle;
        double bLeftPower = powerX + powerY + powerAngle;
        double bRightPower = powerX - powerY - powerAngle;

        Log.d("purepursactions", "set power values " + fLeftPower + " " + fRightPower + " " + bLeftPower + " " + bRightPower);

        double biggestPower = MathFunctions.maxAbsValueDouble(fLeftPower, fRightPower, bLeftPower, bRightPower);
//        opModeUtilities.getTelemetry().addData("biggest power", biggestPower);
        if (biggestPower > 1) {
            fLeftPower /= biggestPower;
            fRightPower /= biggestPower;
            bLeftPower /= biggestPower;
            bRightPower /= biggestPower;
        }
        //92.66611361922297, -235.81741858351225

        Log.d("purespursaction", "target position " + target.getX() + " " + target.getY() + " " + targetAngle);
        driveTrain.setPower(fLeftPower, fRightPower, bLeftPower, bRightPower);

//        opModeUtilities.getTelemetry().addData("current pos", currentPos.toString());
//        opModeUtilities.getTelemetry().addData("power x ", powerX);
//        opModeUtilities.getTelemetry().addData("power y", powerY);
//        opModeUtilities.getTelemetry().addData("distance to target", distanceToTarget);
    }

    @Override
    public boolean checkDoneCondition() {

        lastLine = path.getSegment(path.numSegments() - 1);

        if (odometry.getCurrentPosition().toPoint().distanceTo(lastLine.getFinish()) < 80 //30, 30, 30, 1, 10
                && odometry.getCurrentVelocity().isWithinThreshhold(100, 100, Math.toRadians(5))
                && Math.abs(odometry.getCurrentPosition().getTheta() - preferredAngle) < Math.toRadians(3)) {
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

        follow = path.searchFrom(odometry.getCurrentPosition().toPoint(), radius);

        if (!follow.isPresent()) {
            Log.d("purepursaction", "follow is not present");
            follow = path.searchFrom(odometry.getCurrentPosition().toPoint(), radius);
            radius += 25;
        }

        if (follow.isPresent()) {
            Log.d("purepursaction", "follow present " + follow.get().getX() + " " + follow.get().getY());
            lastLine = path.getSegment(path.numSegments() - 1);
            preferredAngle = lastLine.getHeadingDirection();
            Log.d("purepursaction", "preferred angle " + preferredAngle);
//                opModeUtilities.getTelemetry().addData("preferredAngle", preferredAngle);
//                opModeUtilities.getTelemetry().addData("follow point", follow);
            targetPosition(follow.get(), preferredAngle);
            //Log.d("position", odometry.getCurrentPosition().toString());
            //Log.d("velocity", odometry.getCurrentVelocity().toString());

            radius = 50;
        }

        //if (Thread.interrupted()) throw new InterruptedException();

    }
}
