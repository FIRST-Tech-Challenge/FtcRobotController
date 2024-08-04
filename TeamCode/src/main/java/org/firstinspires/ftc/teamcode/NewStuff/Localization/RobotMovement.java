package org.firstinspires.ftc.teamcode.NewStuff.Localization;

import static org.firstinspires.ftc.teamcode.NewStuff.MathFunctions.absoluteAngleBetweenPoints;
import static org.firstinspires.ftc.teamcode.NewStuff.MathFunctions.angleWrapRad;
import static org.firstinspires.ftc.teamcode.NewStuff.MathFunctions.distance;
import static org.firstinspires.ftc.teamcode.NewStuff.MathFunctions.lineCircleIntersection;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.NewStuff.DriveTrain;
import org.firstinspires.ftc.teamcode.NewStuff.MathFunctions;
import org.firstinspires.ftc.teamcode.NewStuff.OpModeUtilities;
import org.opencv.core.Mat;

import java.util.ArrayList;

public class RobotMovement{
    private final OpModeUtilities opModeUtilities;
    private final Odometry odometry;
    private final DriveTrain driveTrain;

    public RobotMovement (OpModeUtilities opModeUtilities, DriveTrain driveTrain, Odometry odometry) {
        this.opModeUtilities = opModeUtilities;
        this.odometry = odometry;
        this.driveTrain = driveTrain;
    }

    public void targetPosition(Point target, double preferredAngle, double radius){
        Position currentPos = odometry.getCurrentPosition();

        double distanceToTarget = distance(target, currentPos.toPoint());
        double absoluteAngleToPoint = absoluteAngleBetweenPoints(currentPos.toPoint(), target);
        double relativeAngleToPoint = angleWrapRad(absoluteAngleToPoint - currentPos.getTheta());

        PidNav pidX = new PidNav(1 / radius, 0, 0);
        double powerMagnitudeX = Range.clip(pidX.getPower(distanceToTarget), -1, 1);
        double powerX = powerMagnitudeX * Math.cos(relativeAngleToPoint);


        PidNav pidY = new PidNav(1 / radius, 0, 0);
        double powerMagnitudeY = Range.clip(pidY.getPower(distanceToTarget), -1, 1);
        double powerY = powerMagnitudeY * Math.sin(relativeAngleToPoint);


        PidNav pidAngle = new PidNav(1.6 / Math.PI, 0 ,0);
        double powerAngle = Range.clip(pidAngle.getPower(-preferredAngle + relativeAngleToPoint), -1, 1);

        if (distanceToTarget < 0.75 * radius) {
            powerAngle = 0;
        }

        double fLeftPower = powerX - powerY - powerAngle;
        double fRightPower = powerX + powerY + powerAngle;
        double bLeftPower = powerX + powerY - powerAngle;
        double bRightPower = powerX - powerY + powerAngle;

        double biggestPower = Math.max(fLeftPower, Math.max(fRightPower, Math.max(bLeftPower, bRightPower)));
        if (biggestPower > 1) {
            fLeftPower /= biggestPower;
            fRightPower /= biggestPower;
            bLeftPower /= biggestPower;
            bRightPower /= biggestPower;
        }

//        double moveX = pidXNav.getPower(powerX);
//        double moveY = pidYNav.getPower(powerY);
//        double moveAngle = pidAngleNav.getPower(relativeAngleToPoint + preferredAngle) * turnSpeed;
//
//        double relativeTurningAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
//        double moveTurn = Range.clip(relativeTurningAngle / Math.toRadians(30), -1, 1) * turnSpeed;
//
//        double fLeftPower = moveX - moveY - moveAngle;
//        double fRightPower = moveX + moveY + moveAngle;
//        double bLeftPower = moveX + moveY - moveAngle;
//        double bRightPower = moveX - moveY + moveAngle;

       /* double fLeftPower = relativeYtoPoint + relativeXtoPoint;// - moveTurn;
        double fRightPower = relativeYtoPoint - relativeXtoPoint;// + moveTurn;
        double bLeftPower = relativeYtoPoint - relativeXtoPoint;// - moveTurn;
        double bRightPower = relativeYtoPoint + relativeXtoPoint;// + moveTurn;*/

        driveTrain.setPower(fLeftPower, fRightPower, bLeftPower, bRightPower);
        odometry.updatePosition();

        opModeUtilities.getTelemetry().addData("current pos", currentPos.toString());
        opModeUtilities.getTelemetry().addData("power x ", powerX);
        opModeUtilities.getTelemetry().addData("power y", powerY);
        opModeUtilities.getTelemetry().addData("distance to target", distanceToTarget);
        opModeUtilities.getTelemetry().update();
    }

    private Point searchPath(ArrayList<Point> path, double radius) throws ArithmeticException{
        for (int i = path.size() - 1; i >= 1; i--) {
            Point segmentStart = path.get(i - 1);
            Point segmentFinish = path.get(i);
            try {
                return lineCircleIntersection(segmentStart, segmentFinish,
                        odometry.getCurrentPosition().toPoint(), radius);
            } catch (ArithmeticException exception) {
                continue;
            }
        }

        throw new ArithmeticException("no intersection with path found");
    }

    public void pathFollow(ArrayList<Point> path) {
        while (opModeUtilities.getOpMode().opModeIsActive()) {
            Point follow;
            double radius = 300;
            while (true) {
                try {
                    follow = searchPath(path, radius);
                    break;
                } catch (ArithmeticException exception) {
                    radius += 25;
                    continue;
                }
            }

            opModeUtilities.getTelemetry().addData("follow point", follow);
            targetPosition(follow, 0, radius);
        }
    }
}
