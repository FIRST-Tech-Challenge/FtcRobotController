/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.paths.curves;

import org.firstinspires.ftc.teamcode.follower.math.Vector2D;
import org.firstinspires.ftc.teamcode.follower.paths.Piecewise;
import org.firstinspires.ftc.teamcode.follower.paths.TValue;
import java.util.List;

public class CompoundCurve implements Curve {
    private final Piecewise<Curve> curves;

    public CompoundCurve(List<Curve> curves) {
        if (curves.isEmpty()) throw new IllegalArgumentException("Compound curve must have at least one curve.");
        this.curves = new Piecewise<>(Curve::length, curves);
    }

    @Override
    public Vector2D get(double t) {
        TValue.check(t);
        return curves.get(t).get(curves.localT(t));
    }

    @Override
    public double closestParameter(Vector2D position, double initialGuess) {
        double bestT = 0.0;
        double bestDistance = Double.POSITIVE_INFINITY;
        Piecewise.Segment<Curve> guessSegment = curves.getSegment(initialGuess);

        for (Piecewise.Segment<Curve> segment : curves.segments()) {
            Curve curve = segment.value();
            double localGuess = (segment == guessSegment) ? curves.localT(initialGuess) : 0.5;
            double localT = curve.closestParameter(position, localGuess);

            Vector2D point = curve.get(localT);
            double distance = point.distance(position);

            if (distance < bestDistance) {
                bestDistance = distance;
                bestT = curves.globalT(segment, localT);
            }
        }

        return bestT;
    }

    @Override
    public double length() {
        return curves.length();
    }

    @Override
    public double remainingDistance(double t) {
        TValue.check(t);
        return curves.length() - distanceAt(t);
    }

    @Override
    public double parameter(double pathCompletion) {
        return getTFromDistance(pathCompletion * length());
    }

    private double distanceAt(double t) {
        double distanceTraveled = 0.0;
        double cumulativeLength = 0.0;
        double totalLength = curves.length();

        for (Piecewise.Segment<Curve> segment : curves.segments()) {
            Curve curve = segment.value();
            double curveLength = curve.length();
            double currentT = cumulativeLength / totalLength;
            double nextT = (cumulativeLength + curveLength) / totalLength;

            if (t <= currentT) {
                break;
            } else if (t >= nextT) {
                distanceTraveled += curveLength;
            } else {
                double localT = curves.localT(t);
                distanceTraveled += curveLength - curve.remainingDistance(localT);
                break;
            }
            cumulativeLength += curveLength;
        }
        return distanceTraveled;
    }

    public double getTFromDistance(double distance) {
        if (distance <= 0) return 0;
        if (distance >= length()) return 1;

        double remaining = distance;
        for (Piecewise.Segment<Curve> segment : curves.segments()) {
            Curve curve = segment.value();
            double curveLength = curve.length();
            if (remaining <= curveLength) {
                double localT = curve.parameter(remaining / curveLength); // normalize to fraction
                return curves.globalT(segment, localT);
            }
            remaining -= curveLength;
        }
        return 1;
    }

    @Override
    public Vector2D tangent(double t) {
        TValue.check(t);
        return curves.get(t).tangent(curves.localT(t));
    }

    @Override
    public Vector2D derivative(double t) {
        TValue.check(t);
        return curves.get(t).derivative(curves.localT(t));
    }

    @Override
    public double curvature(double t) {
        TValue.check(t);
        return curves.get(t).curvature(curves.localT(t));
    }
}
