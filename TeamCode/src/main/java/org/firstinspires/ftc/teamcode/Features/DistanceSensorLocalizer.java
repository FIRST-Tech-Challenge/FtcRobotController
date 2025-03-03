package org.firstinspires.ftc.teamcode.Features;

import static java.lang.Math.acos;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.tan;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class DistanceSensorLocalizer {

    private final int FIELD_SIZE = 144; // Inches

    private static final double
        SIDE_DIST_OFFSET_X = 0, // Inches
        SIDE_DIST_OFFSET_Y = 0, // Inches
        REAR_DIST_OFFSET_Y = 0; // Inches

    private double
        x_direction_mltplr = 1,
        y_direction_mltplr = 1;

    public Vector2d calculateRealLocation(Pose2d curr_pose,
                                          double rear_Dist,
                                          double side_Dist) {

        if (curr_pose.position.x < 0) x_direction_mltplr = -1;
        if (curr_pose.position.y < 0) y_direction_mltplr = -1;

        double heading = 90 - curr_pose.heading.toDouble();
        double x1 = SIDE_DIST_OFFSET_X / cos(heading);
        double opposite_offset = tan(heading) * SIDE_DIST_OFFSET_X;
        double hypot = hypot(side_Dist, (opposite_offset + SIDE_DIST_OFFSET_Y));
        double point_angle = 180 - acos(hypot/(opposite_offset + SIDE_DIST_OFFSET_Y)) - (90 - heading);
        double x2 = cos(point_angle) * hypot;
        double real_X = (FIELD_SIZE / 2) - x1 + x2;

        double real_Y = (FIELD_SIZE / 2) - cos(heading) * (rear_Dist + REAR_DIST_OFFSET_Y);

        return new Vector2d(x_direction_mltplr * real_X, y_direction_mltplr * real_Y);
    }
}