/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.paths;

import org.firstinspires.ftc.teamcode.follower.config.Modifier;
import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.follower.paths.curves.Curve;
import java.util.List;
import java.util.function.DoubleUnaryOperator;

public final class PathSegment {
    public final Curve curve;
    private final HeadingProvider heading;
    private final List<Modifier> modifiers;
    private final Pose endPose;

    PathSegment(Curve curve, HeadingProvider heading, List<Modifier> modifiers) {
        this.curve = curve;
        this.heading = heading;
        this.modifiers = modifiers;
        endPose = curve.get(1.0).toPose(heading.heading(1.0));
    }

    public double heading(double t) {
        TValue.check(t);
        return heading.heading(t);
    }

    public Pose get(double t) {
        TValue.check(t);
        return curve.get(t).toPose(heading(t));
    }

    List<Modifier> modifiers() {
        return modifiers;
    }

    public Pose endPose() {
        return endPose;
    }

    public interface HeadingProvider {
        double heading(double t);

        static HeadingProvider of(DoubleUnaryOperator heading) {
            return heading::applyAsDouble;
        }
    }
}
