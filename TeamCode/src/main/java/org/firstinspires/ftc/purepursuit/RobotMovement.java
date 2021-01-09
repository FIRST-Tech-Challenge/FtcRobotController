package org.firstinspires.ftc.purepursuit;

import org.firstinspires.ftc.robot.Range;

import static org.firstinspires.ftc.robot.Robot.*;
import static org.firstinspires.ftc.purepursuit.MathFunctions.angleWrap;
import static org.firstinspires.ftc.robot_utilities.MovementVars.*;

public class RobotMovement {

    public static void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed) {

        double distanceToTarget = Math.hypot(y - worldYPosition, x - worldXPosition);
        double absoluteAngleToTarget = Math.atan2(y - worldYPosition, x - worldXPosition);

        double relativeAngleToPoint = angleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));


        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        final double v = (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint))/movementSpeed;
        movement_x = relativeXToPoint / v;
        movement_y = relativeYToPoint / v;

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;

        movement_turn = Range.clip(relativeTurnAngle/Math.toRadians(30), -1, 1) * turnSpeed;
    }
}
