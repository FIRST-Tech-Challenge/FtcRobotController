package org.firstinspires.ftc.teamcode.NewStuff.Navigation;

import static org.firstinspires.ftc.teamcode.NewStuff.MathFunctions.angleWrapRad;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NewStuff.DriveTrain;

public class RobotMovement{
    private Telemetry jose;
    private Odometry odometry;
    private DriveTrain driveTrain;
    private PidNav pidXNav;
    private PidNav pidYNav;
    private PidNav pidAngleNav;

    public RobotMovement (Telemetry telemetry, DriveTrain driveTrain, Odometry odometry, PidNav pidNav) {
        this.jose = telemetry;
        this.odometry = odometry;
        this.driveTrain = driveTrain;
        this.pidXNav = new PidNav(0.1, 0, 0);
        this.pidYNav = new PidNav(0.1, 0, 0);
        this.pidAngleNav = new PidNav(0.1, 0, 0);
    }

    public void goToPosition(double x, double y, double preferredAngle, double turnSpeed){
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
        double turn = Range.clip(relativeTurningAngle / Math.toRadians(30), -1, 1) * turnSpeed;
        /*double fLeftPower = moveX - moveY;
        double fRightPower = moveX - moveY;
        double bLeftPower = moveX + moveY;
        double bRightPower = moveX + moveY;*/

        double fLeftPower = relativeYtoPoint - relativeXtoPoint - turn;
        double fRightPower = relativeYtoPoint - relativeXtoPoint + turn;
        double bLeftPower = relativeYtoPoint + relativeXtoPoint - turn;
        double bRightPower = relativeYtoPoint + relativeXtoPoint + turn;

        driveTrain.setPower(fLeftPower, fRightPower, bLeftPower, bRightPower);

        odometry.updatePosition();
        jose.update();
    }
}
