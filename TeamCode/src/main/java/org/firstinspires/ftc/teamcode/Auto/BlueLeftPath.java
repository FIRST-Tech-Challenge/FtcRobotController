package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class BlueLeftPath {
    PathChain path;

    public BlueLeftPath() {
        PathBuilder builder = new PathBuilder();

        builder
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(8.000, 108.000, Point.CARTESIAN),
                                new Point(18.000, 118.000, Point.CARTESIAN),
                                new Point(18.000, 126.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(18.000, 126.000, Point.CARTESIAN),
                                new Point(30.000, 120.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(30.000, 120.000, Point.CARTESIAN),
                                new Point(18.000, 126.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(18.000, 126.000, Point.CARTESIAN),
                                new Point(32.000, 128.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(10))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(32.000, 128.000, Point.CARTESIAN),
                                new Point(18.000, 126.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(10), Math.toRadians(-45))
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(18.000, 126.000, Point.CARTESIAN),
                                new Point(30.000, 130.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(30))
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(30.000, 130.000, Point.CARTESIAN),
                                new Point(18.000, 126.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(27), Math.toRadians(-45));

        path = builder.build();

    }

    public PathChain getPath() {
        return path;
    }
}
