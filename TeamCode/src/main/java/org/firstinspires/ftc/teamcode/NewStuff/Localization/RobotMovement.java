package org.firstinspires.ftc.teamcode.NewStuff.Localization;

import static org.firstinspires.ftc.teamcode.NewStuff.Math.MathFunctions.angleWrapRad;
import static org.firstinspires.ftc.teamcode.NewStuff.Math.MathFunctions.maxAbsValueDouble;

import android.util.Log;

import androidx.annotation.GuardedBy;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.NewStuff.DriveTrain;
import org.firstinspires.ftc.teamcode.NewStuff.Math.Path;
import org.firstinspires.ftc.teamcode.NewStuff.Math.Point;
import org.firstinspires.ftc.teamcode.NewStuff.Math.Position;
import org.firstinspires.ftc.teamcode.NewStuff.Math.Segment;
import org.firstinspires.ftc.teamcode.NewStuff.Math.Vector;
import org.firstinspires.ftc.teamcode.NewStuff.OpModeUtilities;

import java.util.Optional;
import java.util.concurrent.locks.ReentrantLock;

public class RobotMovement{
    private final OpModeUtilities opModeUtilities;
    private final Odometry odometry;
    private final DriveTrain driveTrain;
    @GuardedBy("lock")
    private final PidNav pidX;
    @GuardedBy("lock")
    private final PidNav pidY;
    @GuardedBy("lock")
    private final PidNav pidAngle;

    private final ReentrantLock lock = new ReentrantLock();

    public RobotMovement (OpModeUtilities opModeUtilities, DriveTrain driveTrain, Odometry odometry) {
        this.opModeUtilities = opModeUtilities;
        this.odometry = odometry;
        this.driveTrain = driveTrain;
        this.pidX = new PidNav(1. / 300, 0, 0);
        this.pidY = new PidNav(1. / 300, 0, 0);
        this.pidAngle = new PidNav(3 / 3.14, 0, 0);
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
        double angleError = angleWrapRad(targetAngle - currentPos.getTheta());
        double directionError = angleWrapRad(targetDirection - currentPos.getTheta());

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

        double biggestPower = maxAbsValueDouble(fLeftPower, fRightPower, bLeftPower, bRightPower);
//        opModeUtilities.getTelemetry().addData("biggest power", biggestPower);
        if (biggestPower > 1) {
            fLeftPower /= biggestPower;
            fRightPower /= biggestPower;
            bLeftPower /= biggestPower;
            bRightPower /= biggestPower;
        }

        driveTrain.setPower(fLeftPower, fRightPower, bLeftPower, bRightPower);

//        opModeUtilities.getTelemetry().addData("current pos", currentPos.toString());
//        opModeUtilities.getTelemetry().addData("power x ", powerX);
//        opModeUtilities.getTelemetry().addData("power y", powerY);
//        opModeUtilities.getTelemetry().addData("distance to target", distanceToTarget);
    }
    //Maybe do a PID reset if it breaks
    public void pathFollow(Path path) throws InterruptedException {
        lock.lock();
        try {
            while (true) {
                Optional<Point> follow;
                double radius = 300;
                do {
                    follow = path.searchFrom(odometry.getCurrentPosition().toPoint(), radius);
                    radius += 25;
                } while (!follow.isPresent());
                Segment lastLine = path.getSegment(path.numSegments() - 1);
                double preferredAngle = lastLine.getHeadingDirection();
//                opModeUtilities.getTelemetry().addData("preferredAngle", preferredAngle);
//                opModeUtilities.getTelemetry().addData("follow point", follow);
                targetPosition(follow.get(), preferredAngle);
                Log.d("position", odometry.getCurrentPosition().toString());
                Log.d("velocity", odometry.getCurrentVelocity().toString());
                if (odometry.getCurrentPosition().toPoint().distanceTo(lastLine.getFinish()) < 30
                        && odometry.getCurrentVelocity().isWithinThreshhold(30, 30, Math.toRadians(1))
                        && Math.abs(odometry.getCurrentPosition().getTheta() - preferredAngle) < Math.toRadians(2)) {
                    opModeUtilities.getTelemetry().addLine("breake");
                    opModeUtilities.getTelemetry().update();
                    driveTrain.setPower(0, 0, 0, 0);
                    break;
                }

                if (Thread.interrupted()) throw new InterruptedException();
            }
        } finally {
            lock.unlock();
        }
    }
}
