package org.firstinspires.ftc.teamcode.NewStuff.actions;

import android.util.Log;

import androidx.annotation.GuardedBy;

import com.kalipsorobotics.fresh.localization.Odometry;
import com.kalipsorobotics.fresh.localization.PidNav;
import com.kalipsorobotics.fresh.math.MathFunctions;
import com.kalipsorobotics.fresh.math.Path;
import com.kalipsorobotics.fresh.math.Point;
import com.kalipsorobotics.fresh.math.Position;
import com.kalipsorobotics.fresh.math.Segment;
import com.kalipsorobotics.fresh.math.Vector;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.NewStuff.modules.DriveTrain;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class PurePursuitAction extends Action {

    List<Point> pathPoints = new ArrayList<Point>();
    DriveTrain driveTrain;
    Odometry odometry;

    @GuardedBy("lock")
    private final PidNav pidX;
    @GuardedBy("lock")
    private final PidNav pidY;
    @GuardedBy("lock")
    private final PidNav pidAngle;

    Path path;
    Segment lastLine;
    double preferredAngle;
    double radius;
    Optional<Point> follow;

    public PurePursuitAction(DriveTrain driveTrain, Odometry odometry, Action dependentAction) {
        this.driveTrain = driveTrain;
        this.odometry = odometry;

        this.pidX = new PidNav(1. / 300, 0, 0);
        this.pidY = new PidNav(1. / 300, 0, 0);
        this.pidAngle = new PidNav(3 / 3.14, 0, 0);
        Log.d("purepursaction", "constructed");

        this.dependentAction = dependentAction;
    }

    public PurePursuitAction(DriveTrain driveTrain, Odometry odometry) {
        this.driveTrain = driveTrain;
        this.odometry = odometry;

        this.pidX = new PidNav(1. / 300, 0, 0);
        this.pidY = new PidNav(1. / 300, 0, 0);
        this.pidAngle = new PidNav(3 / 3.14, 0, 0);
        Log.d("purepursaction", "constructed");

        this.dependentAction = new DoneStateAction();
    }

    public void addPoint(int x, int y) {
        pathPoints.add(new Point(x, y));
        Log.d("purepursaction", "added point " + x + ", " + y);
    }

    @GuardedBy("lock")
    private void targetPosition(Point target, double preferredAngle) {
        Position currentPos = odometry.getCurrentPosition();
        Vector currentToTarget = Vector.between(currentPos.toPoint(), target);

        double distanceToTarget = currentToTarget.getLength();
        double targetAngle = currentToTarget.getHeadingDirection();
        double targetDirection = currentToTarget.getHeadingDirection();

        if (distanceToTarget < 100) {
            targetAngle = preferredAngle;
        }
        double angleError = MathFunctions.angleWrapRad(targetAngle - currentPos.getTheta());
        double directionError = MathFunctions.angleWrapRad(targetDirection - currentPos.getTheta());

//        opModeUtilities.getTelemetry().addData("targetAngle", targetAngle);
//        opModeUtilities.getTelemetry().addData("angle error", angleError);
//        opModeUtilities.getTelemetry().addData("direction error", directionError);

        double xError = Math.cos(directionError) * distanceToTarget;
        double powerX = Range.clip(pidX.getPower(xError), -1, 1);

        double yError = Math.sin(directionError) * distanceToTarget;
        double powerY = Range.clip(pidY.getPower(yError), -1, 1);

        double powerAngle = Range.clip(pidAngle.getPower(angleError), -1, 1);
//        opModeUtilities.getTelemetry().addData("power angle", powerAngle);

        double fLeftPower = powerX - powerY - powerAngle;
        double fRightPower = powerX + powerY + powerAngle;
        double bLeftPower = powerX + powerY - powerAngle;
        double bRightPower = powerX - powerY + powerAngle;

        double biggestPower = MathFunctions.maxAbsValueDouble(fLeftPower, fRightPower, bLeftPower, bRightPower);
//        opModeUtilities.getTelemetry().addData("biggest power", biggestPower);
        if (biggestPower > 1) {
            fLeftPower /= biggestPower;
            fRightPower /= biggestPower;
            bLeftPower /= biggestPower;
            bRightPower /= biggestPower;
        }

        driveTrain.setPower(fLeftPower, fRightPower, bLeftPower, bRightPower);

        Log.d("purepursaction", "set target and set powers " + fLeftPower + " " + fRightPower + " " + bLeftPower + " " + bRightPower);

//        opModeUtilities.getTelemetry().addData("current pos", currentPos.toString());
//        opModeUtilities.getTelemetry().addData("power x ", powerX);
//        opModeUtilities.getTelemetry().addData("power y", powerY);
//        opModeUtilities.getTelemetry().addData("distance to target", distanceToTarget);
    }

    @Override
    public boolean checkDoneCondition() {

        lastLine = path.getSegment(path.numSegments() - 1);

        if (odometry.getCurrentPosition().toPoint().distanceTo(lastLine.getFinish()) < 30
                && odometry.getCurrentVelocity().isWithinThreshhold(30, 30, Math.toRadians(1))
                && Math.abs(odometry.getCurrentPosition().getTheta() - preferredAngle) < Math.toRadians(2)) {
            //opModeUtilities.getTelemetry().addLine("breake");
            //opModeUtilities.getTelemetry().update();
            driveTrain.setPower(0, 0, 0, 0);
            Log.d("purepursaction", "done");
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void update() {
        if(!hasStarted) {
            path = new Path(pathPoints);
            Log.d("purepursaction", "set path");
            hasStarted = true;
        }

        Log.d("purepursaction", "entered update");

        follow = path.searchFrom(odometry.getCurrentPosition().toPoint(), radius);

        if (!follow.isPresent()) {
            Log.d("purepursaction", "follow is not present");
            follow = path.searchFrom(odometry.getCurrentPosition().toPoint(), radius);
            radius += 25;
        }

        if(follow.isPresent()) {
            Log.d("purepursaction", "follow present: radius " + radius);
            lastLine = path.getSegment(path.numSegments() - 1);
            preferredAngle = lastLine.getHeadingDirection();
//                opModeUtilities.getTelemetry().addData("preferredAngle", preferredAngle);
//                opModeUtilities.getTelemetry().addData("follow point", follow);
            targetPosition(follow.get(), preferredAngle);
            Log.d("position", odometry.getCurrentPosition().toString());
            //Log.d("velocity", odometry.getCurrentVelocity().toString());

            radius = 100;
        }

        //if (Thread.interrupted()) throw new InterruptedException();

    }
}
