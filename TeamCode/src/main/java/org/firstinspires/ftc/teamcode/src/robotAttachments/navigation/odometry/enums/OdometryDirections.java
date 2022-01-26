package org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.odometry.enums;

import java.util.HashMap;

/**
 * A enum to correlate relative terms of measure to absolute angles
 */
public enum OdometryDirections {
    Forward,
    Backward,
    Right,
    Left;

    /**
     * A hashmap that contains the data to make the substitution
     */
    public static final HashMap<OdometryDirections, Double> positionToAngle = new HashMap<OdometryDirections, Double>() {{
        put(OdometryDirections.Forward, 0D);
        put(OdometryDirections.Backward, 180D);
        put(OdometryDirections.Right, 90D);
        put(OdometryDirections.Left, 270D);
    }};
}
