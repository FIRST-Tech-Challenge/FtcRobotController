/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.paths.interpolator;

import static org.firstinspires.ftc.teamcode.follower.utils.Angle.error;

import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.follower.math.Vector2D;
import org.firstinspires.ftc.teamcode.follower.paths.curves.Curve;
import org.firstinspires.ftc.teamcode.follower.utils.Angle;
import java.util.function.BiFunction;

@FunctionalInterface
public interface Interpolator {
    Interpolator tangent = Interpolator.geometric((curve, t) -> curve.tangent(t).theta());

    default Interpolator reverse() {
        Interpolator outer = this;
        return (curve, t) -> Angle.normalize(outer.interpolate(curve, t) + Math.PI);
    }

    static Interpolator constant(double heading) {
        double finalHeading = Angle.normalize(heading);
        return (Curve curve, double t) -> finalHeading;
    }

    static Interpolator constant(Pose pose) {
        return constant(pose.heading());
    }

    static Interpolator linear(double start, double end) {
        double finalStart = Angle.normalize(start);
        double finalEnd = Angle.normalize(end);
        double deltaHeading = error(finalStart, finalEnd);

        return (curve, t) -> Angle.normalize(finalStart + deltaHeading * t);
    }

    static Interpolator linear(Pose start, Pose end) {
        return linear(start.heading(), end.heading());
    }

    static Interpolator longLinear(double start, double end) {
        double finalStart = Angle.normalize(start);
        double finalEnd = Angle.normalize(end);

        double deltaHeading = -Angle.turnDirection(finalStart, finalEnd)
                * (2 * Math.PI - Angle.smallestDifference(finalStart, finalEnd));

        return (curve, t) -> Angle.normalize(finalStart + deltaHeading * t);
    }

    static Interpolator longLinear(Pose start, Pose end) {
        return longLinear(start.heading(), end.heading());
    }

    static Interpolator facingPoint(Vector2D point) {
        return Interpolator.geometric((curve, t) -> point.minus(curve.get(t)).theta());
    }

    static Interpolator facingPoint(Pose pose) {
        return facingPoint(pose.toVector2D());
    }

    static PiecewiseInterpolator piecewise() {
        return new PiecewiseInterpolator();
    }

    static Interpolator geometric(BiFunction<Curve, Double, Double> function) {
        return new Interpolator() {
            @Override
            public double interpolate(Curve curve, double t) {
                return function.apply(curve, t);
            }

            @Override
            public boolean isGeometric() {
                return true;
            }
        };
    }

    double interpolate(Curve curve, double t);

    default boolean isGeometric() {
        return false;
    }
}
