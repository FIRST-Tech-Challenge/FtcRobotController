package org.firstinspires.ftc.teamcode.robots.csbot.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.Arrays;
import java.util.List;

/**
 * Includes utility methods for two wheel differential drivetrain inverse kinematics.
 * @author Mahesh Natamai
 */

public class DiffyKinematics {

    /**
     * Computes the wheel velocities corresponding to [robotVel] given [trackWidth] and [chassisLength].
     *
     * @param robotVel velocity of the robot in its reference frame
     * @param trackWidth lateral distance between the differential wheel pair
     */
    public static List<Double> robotToWheelVelocities(Pose2d robotVel, double trackWidth) {
        assert Utils.approxEquals(robotVel.getY(), 0) : "Lateral (robot y) velocity must be zero for differential drives";

        return Arrays.asList(
                robotVel.getX() - trackWidth / 2 * robotVel.getHeading(),
                robotVel.getX() + trackWidth / 2 * robotVel.getHeading()
        );
    }

    /**
     * Computes the wheel accelerations corresponding to [robotAccel] given the provided [trackWidth] and
     * [chassisLength].
     *
     * @param robotAccel velocity of the robot in its reference frame
     * @param trackWidth lateral distance between the differential wheel pair
     */
    public static List<Double> robotToWheelAccelerations(Pose2d robotAccel, double trackWidth) {
        //todo this is not accellerations
        return robotToWheelVelocities(robotAccel, trackWidth);
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
