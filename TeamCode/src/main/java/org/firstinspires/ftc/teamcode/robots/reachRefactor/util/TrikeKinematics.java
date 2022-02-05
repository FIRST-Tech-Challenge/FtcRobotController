package org.firstinspires.ftc.teamcode.robots.reachRefactor.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.Arrays;
import java.util.List;

/**
 * Includes utility methods for trike drivetrain inverse kinematics.
 * @author Mahesh Natamai
 */

public class TrikeKinematics {

    /**
     * Computes the wheel velocities corresponding to [robotVel] given [trackWidth] and [chassisLength].
     *
     * @param robotVel velocity of the robot in its reference frame
     * @param trackWidth lateral distance between the differential wheel pair
     */
    public static List<Double> robotToWheelVelocities(Pose2d robotVel, double trackWidth, double chassisLength) {
        assert Utils.approxEquals(robotVel.getY(), 0) : "Lateral (robot y) velocity must be zero for trike drives";

        return Arrays.asList(
                robotVel.getX() - trackWidth / 2 * robotVel.getHeading(),
                robotVel.getX() + trackWidth / 2 * robotVel.getHeading(),
                Math.hypot(robotVel.getX(), chassisLength * robotVel.getHeading())
        );
    }

    /**
     * Computes the wheel accelerations corresponding to [robotAccel] given the provided [trackWidth] and
     * [chassisLength].
     *
     * @param robotAccel velocity of the robot in its reference frame
     * @param trackWidth lateral distance between the differential wheel pair
     * @param chassisLength distance between the front (differential) and back (swerve) wheels
     */
    public static List<Double> robotToWheelAccelerations(Pose2d robotAccel, double trackWidth, double chassisLength) {
        return robotToWheelVelocities(robotAccel, trackWidth, chassisLength);
    }

    /**
     * Computes the swivel module orientation (in radians) corresponding to [robotVel] given the provided
     * [chassisLength].
     *
     * @param robotVel velocity of the robot in its reference frame
     * @param chassisLength distance between the front (differential) and back (swerve) wheels
     */
    public static double robotToSwivelAngle(Pose2d robotVel, double chassisLength) {
        assert Utils.approxEquals(robotVel.getY(), 0) : "Lateral (robot y) velocity must be zero for trike drives";

        return Utils.wrapAngleRad(Math.PI / 2 + Math.atan2(-chassisLength * robotVel.getHeading(), robotVel.getX()));
    }

    /**
     * Computes the robot velocities corresponding to [wheelVelocities] and the given drive parameters.
     *
     * @param wheelVelocities wheel velocities (or wheel position deltas)
     * @param trackWidth lateral distance between the differential wheel pair
     */
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
