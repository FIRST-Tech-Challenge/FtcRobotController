package org.firstinspires.ftc.teamcode.NewStuff.Navigation;

import static org.firstinspires.ftc.teamcode.MathFunctions.angleWrapRad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;

public class RobotMovement{
    private Telemetry jose;
    private Robot robot;
    private Odometry odometry;
    private PidNav pidXNav;
    private PidNav pidYNav;
    private PidNav pidAngleNav;

    public RobotMovement (Telemetry telemetry, Robot robot, Odometry odometry, PidNav pidNav) {
        this.jose = telemetry;
        this.robot = robot;
        this.odometry = odometry;
        this.pidXNav = new PidNav(0.1, 0, 0);
        this.pidYNav = new PidNav(0.1, 0, 0);
        this.pidAngleNav = new PidNav(0.1, 0, 0);
    }

    public void goToPosition(double x, double y, double preferredAngle){
        Position currentPos = odometry.getCurrentPosition();
        double distanceToTarget = Math.hypot(x - currentPos.getX(), currentPos.getY());
        double absoluteAngleToTarget = Math.atan2(y - currentPos.getY(), x - currentPos.getX());
        double relativeAngleToPoint =
                angleWrapRad(absoluteAngleToTarget - (currentPos.getTheta() - Math.toRadians(90)));
        jose.addData("current pos", currentPos.toString());
        double relativeXtoPoint = Math.cos(relativeAngleToPoint);// * distanceToTarget;
        double relativeYtoPoint = Math.sin(relativeAngleToPoint);// * distanceToTarget;
        jose.addData("relative x ", relativeXtoPoint);
        jose.addData("relative y", relativeYtoPoint);

//        double moveX = pidXNav.getPower(relativeXtoPoint);
//        double moveY = pidYNav.getPower(relativeYtoPoint);
//        double moveAngle = pidAngleNav.getPower(relativeAngleToPoint);

        double relativeTurningAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
        /*double fLeftPower = moveX - moveY;
        double fRightPower = moveX - moveY;
        double bLeftPower = moveX + moveY;
        double bRightPower = moveX + moveY;*/

        double fLeftPower = relativeYtoPoint - relativeXtoPoint;
        double fRightPower = relativeYtoPoint - relativeXtoPoint;
        double bLeftPower = relativeYtoPoint + relativeXtoPoint;
        double bRightPower = relativeYtoPoint + relativeXtoPoint;
        robot.setPower(fLeftPower, fRightPower, bLeftPower, bRightPower);
        odometry.updatePosition();
        jose.update();
    }
}
