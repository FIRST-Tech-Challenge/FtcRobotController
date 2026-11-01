/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.paths.curves;

import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.follower.math.Vector2D;
import org.firstinspires.ftc.teamcode.follower.paths.TValue;
import org.firstinspires.ftc.teamcode.follower.utils.Utils;

public class Line implements Curve {
    private final Vector2D start;
    private final Vector2D end;
    private final double length;
    private final Vector2D displacement;
    private final Vector2D tangent;

    public Line(Pose start, Pose end) {
        this(start.toVector2D(), end.toVector2D());
    }

    public Line(Vector2D start, Vector2D end) {
        this.start = start;
        this.end = end;
        length = start.distance(end);
        displacement = end.minus(start);

        if (end.minus(start).magnitude() == 0) {
            throw new IllegalArgumentException("Line cannot have zero length");
        }

        tangent = displacement.normalized();
    }

    @Override
    public Vector2D startPoint() {
        return start;
    }

    @Override
    public Vector2D endPoint() {
        return end;
    }

    @Override
    public Vector2D get(double t) {
        TValue.check(t);
        return start.plus(displacement.times(t));
    }

    @Override
    public Vector2D tangent(double t) {
        TValue.check(t);
        return tangent;
    }

    @Override
    public double curvature(double t) {
        TValue.check(t);
        return 0;
    }

    @Override
    public double closestParameter(Vector2D position, double initialGuess) {
        Vector2D PA = position.minus(start);
        return Utils.clamp(displacement.dot(PA) / Math.pow(displacement.magnitude(), 2), 0, 1);
    }

    @Override
    public double length() {
        return length;
    }

    @Override
    public Vector2D derivative(double t) {
        TValue.check(t);
        return displacement;
    }
}
