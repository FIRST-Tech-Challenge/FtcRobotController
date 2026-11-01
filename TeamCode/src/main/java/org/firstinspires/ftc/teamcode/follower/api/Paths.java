/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.api;

import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.follower.math.Vector2D;
import org.firstinspires.ftc.teamcode.follower.paths.AtomicPath;
import org.firstinspires.ftc.teamcode.follower.paths.CompoundPath;
import org.firstinspires.ftc.teamcode.follower.paths.Path;
import org.firstinspires.ftc.teamcode.follower.paths.curves.Curve;
import org.firstinspires.ftc.teamcode.follower.paths.curves.Line;
import org.firstinspires.ftc.teamcode.follower.paths.curves.bezier.BezierCurve;

public final class Paths {
    private Paths() {}

    /**
     * Creates a path from multiple paths.
     */
    public static Path path(Path... paths) {
        return new CompoundPath(paths);
    }

    /**
     * Creates a path from a curve.
     */
    public static Path path(Curve curve) {
        return new AtomicPath(curve);
    }

    /**
     * Creates a straight line path from the start point to the end point.
     */
    public static Path line(Vector2D start, Vector2D end) {
        return path(new Line(start, end));
    }

    /**
     * Creates a straight line path from the start pose to the end pose.
     */
    public static Path line(Pose start, Pose end) {
        return path(new Line(start, end));
    }

    /**
     * Creates a Bézier curve.
     * Requires at least 2 control poses.
     * The first and last poses are the start and end of the curve, while the intermediate poses are control poses
     */
    public static Path curve(Pose... poses) {
        return path(new BezierCurve(poses));
    }

    /**
     * Creates a Bézier curve.
     * Requires at least 2 control points.
     * The first and last points are the start and end of the curve, while the intermediate points are control points
     */
    public static Path curve(Vector2D... points) {
        return path(new BezierCurve(points));
    }

    /**
     * Generates a path with a Bézier curve through the input poses.
     * Requires at least 2 poses.
     */
    public static Path through(Pose... poses) {
        if (poses.length == 2) return line(poses[0], poses[1]);
        return path(BezierCurve.through(poses));
    }
}
