package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.objects.Subsystem;

/**
 * Something that knows which way the robot is facing.
 *
 * <p>The drive only needs a heading. Position on the field (x, y) arrives later with
 * odometry and pathing (PORT-PLAN piece P5), which will add to this interface.
 */
public interface Imu extends Subsystem {
    /** @return true once the heading can be trusted */
    boolean Ready();

    /**
     * @return how far the robot has turned since the last reset, in degrees,
     *     counter-clockwise positive, between -180 and 180
     */
    double GetYaw();

    /** Make the direction the robot faces right now count as 0 degrees. */
    void ResetYaw();
}
