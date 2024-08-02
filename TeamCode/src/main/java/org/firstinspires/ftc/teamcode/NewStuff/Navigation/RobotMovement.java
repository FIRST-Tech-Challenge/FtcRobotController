package org.firstinspires.ftc.teamcode.NewStuff.Navigation;

import static org.firstinspires.ftc.teamcode.NewStuff.MathFunctions.angleWrapRad;
import static org.firstinspires.ftc.teamcode.NewStuff.MathFunctions.lineCircleIntersection;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NewStuff.DriveTrain;
import org.firstinspires.ftc.teamcode.NewStuff.OpModeUtilities;

import java.util.ArrayList;

public class RobotMovement{
    private OpModeUtilities opModeUtilities;
    private Odometry odometry;
    private DriveTrain driveTrain;
    private PidNav pidXNav;
    private PidNav pidYNav;
    private PidNav pidAngleNav;

    public RobotMovement (OpModeUtilities opModeUtilities, DriveTrain driveTrain, Odometry odometry, PidNav pidNav) {
        this.opModeUtilities = opModeUtilities;
        this.odometry = odometry;
        this.driveTrain = driveTrain;
        this.pidXNav = new PidNav(0.1, 0, 0);
        this.pidYNav = new PidNav(0.1, 0, 0);
        this.pidAngleNav = new PidNav(0.1, 0, 0);
    }

    public void targetPosition(Point follow, double preferredAngle, double turnSpeed) {
        targetPosition(follow.getX(), follow.getY(), preferredAngle, turnSpeed);
    }

    public void targetPosition(double x, double y, double preferredAngle, double turnSpeed){
        Position currentPos = odometry.getCurrentPosition();
        double distanceToTarget = Math.hypot(x - currentPos.getX(), currentPos.getY());
        double absoluteAngleToTarget = Math.atan2(y - currentPos.getY(), x - currentPos.getX());
        double relativeAngleToPoint =
                angleWrapRad(absoluteAngleToTarget - (currentPos.getTheta() - Math.toRadians(90)));
        opModeUtilities.getTelemetry().addData("current pos", currentPos.toString());
        double relativeXtoPoint = Math.cos(relativeAngleToPoint);// * distanceToTarget;
        double relativeYtoPoint = Math.sin(relativeAngleToPoint);// * distanceToTarget;
        opModeUtilities.getTelemetry().addData("relative x ", relativeXtoPoint);
        opModeUtilities.getTelemetry().addData("relative y", relativeYtoPoint);

//        double moveX = pidXNav.getPower(relativeXtoPoint);
//        double moveY = pidYNav.getPower(relativeYtoPoint);
//        double moveAngle = pidAngleNav.getPower(relativeAngleToPoint);

        double relativeTurningAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
        double turn = Range.clip(relativeTurningAngle / Math.toRadians(30), -1, 1) * turnSpeed;
        /*double fLeftPower = moveX - moveY;
        double fRightPower = moveX - moveY;
        double bLeftPower = moveX + moveY;
        double bRightPower = moveX + moveY;*/

        double fLeftPower = relativeYtoPoint - relativeXtoPoint;// - turn;
        double fRightPower = relativeYtoPoint - relativeXtoPoint;// + turn;
        double bLeftPower = relativeYtoPoint + relativeXtoPoint;// - turn;
        double bRightPower = relativeYtoPoint + relativeXtoPoint;// + turn;

        driveTrain.setPower(fLeftPower, fRightPower, bLeftPower, bRightPower);

        odometry.updatePosition();
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
            double radius = 100;
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
            targetPosition(follow, 0, 0);
        }
    }
}
