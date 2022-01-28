package org.firstinspires.ftc.teamcode.robots.reachRefactor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.UtilMethods;

import java.util.Arrays;
import java.util.List;

public class TrikeKinematics {
    public static List<Double> robotToWheelVelocities(Pose2d robotVel, double trackWidth, double chassisLength) {
        assert UtilMethods.approxEquals(robotVel.getY(), 0) : "Lateral (robot y) velocity must be zero for trike drives";

        return Arrays.asList(
                robotVel.getX() - trackWidth / 2 * robotVel.getHeading(),
                robotVel.getX() + trackWidth / 2 * robotVel.getHeading(),
                Math.hypot(robotVel.getX(), chassisLength * robotVel.getHeading())
        );
    }

    public static double robotToSwivelAngle(Pose2d robotVel, double chassisLength) {
        assert UtilMethods.approxEquals(robotVel.getY(), 0) : "Lateral (robot y) velocity must be zero for trike drives";

        return UtilMethods.wrapAngle(Math.PI / 4 + Math.atan2(chassisLength * robotVel.getHeading(), robotVel.getX()));
    }

    public static List<Double> robotToWheelAccelerations(Pose2d robotAccel, double trackWidth, double chassisLength) {
        return robotToWheelVelocities(robotAccel, trackWidth, chassisLength);
    }

    public static Pose2d wheelToRobotVelocities(List<Double> wheelVelocities, double trackWidth) {
        double left = wheelVelocities.get(0);
        double right = wheelVelocities.get(1);
        return new Pose2d(
                (left + right) / 2.0,
                0.0,
                (-left + right) / trackWidth
        );
    }
}
