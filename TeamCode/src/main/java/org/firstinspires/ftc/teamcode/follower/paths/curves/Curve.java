/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.paths.curves;

import org.firstinspires.ftc.teamcode.follower.math.Vector2D;
import org.firstinspires.ftc.teamcode.follower.paths.TValue;

public interface Curve {
    Vector2D get(double t);

    double closestParameter(Vector2D position, double initialGuess);

    default double closestParameter(Vector2D position) {
        return closestParameter(position, 0.5);
    }

    double length();

    default double remainingDistance(double t) {
        TValue.check(t);
        return (1 - t) * length();
    }
    ;

    default double pathCompletion(double t) {
        if (length() == 0) return 0.0;
        return 1 - remainingDistance(t) / length();
    }

    default double parameter(double pathCompletion) {
        TValue.check(pathCompletion);
        return pathCompletion;
    }

    Vector2D derivative(double t);

    /**
     * Normalized
     */
    default Vector2D tangent(double t) {
        return derivative(t).normalized();
    }

    double curvature(double t);

    default Vector2D leftNormal(double t) {
        Vector2D tangent = tangent(t);
        return Vector2D.cartesian(-tangent.y(), tangent.x());
    }

    default Vector2D endPoint() {
        return get(1.0);
    }

    default Vector2D startPoint() {
        return get(0.0);
    }
}
