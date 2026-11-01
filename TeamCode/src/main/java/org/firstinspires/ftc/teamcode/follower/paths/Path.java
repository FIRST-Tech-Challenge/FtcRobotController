/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.paths;

import static org.firstinspires.ftc.teamcode.follower.utils.Utils.*;

import org.firstinspires.ftc.teamcode.follower.config.Modifier;
import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.follower.math.Vector2D;
import org.firstinspires.ftc.teamcode.follower.paths.curves.Curve;
import org.firstinspires.ftc.teamcode.follower.paths.interpolator.Interpolator;
import java.util.List;

public abstract class Path {
    public final Curve curve;
    private Pose endPose;
    public final List<Modifier> modifiers;

    public Path(Curve curve, List<Modifier> modifiers) {
        this.curve = curve;
        this.modifiers = copyOf(modifiers);
    }

    public abstract double heading(double t);

    protected abstract boolean hasHeading();

    public final List<PathSegment> getSegments() {
        if (!hasHeading()) throw new IllegalStateException("Cannot resolve segments: path has no heading.");
        return getSegments(PathSegment.HeadingProvider.of(this::heading), listOf());
    }

    public Pose endPose() {
        if (endPose == null) {
            endPose = curve.get(1.0).toPose(heading(1.0));
        }

        return endPose;
    }

    public Pose get(double t) {
        TValue.check(t);
        return curve.get(t).toPose(heading(t));
    }

    protected abstract List<PathSegment> getSegments(
            PathSegment.HeadingProvider parentHeading, List<Modifier> modifiers);

    public Path heading(Interpolator interpolator) {
        return withHeading(interpolator);
    }

    protected abstract Path withHeading(Interpolator interpolator);

    public Path with(List<Modifier> modifiers) {
        return withModifiers(concat(this.modifiers, modifiers));
    }

    public Path with(Modifier... modifiers) {
        return with(listOf(modifiers));
    }

    protected abstract Path withModifiers(List<Modifier> modifiers);

    public Path constant(double heading) {
        return heading(Interpolator.constant(heading));
    }

    public Path constant(Pose pose) {
        return constant(pose.heading());
    }

    public Path linear(double start, double end) {
        return heading(Interpolator.linear(start, end));
    }

    public Path linear(Pose start, Pose end) {
        return linear(start.heading(), end.heading());
    }

    public Path linear(double start, double end, double endT) {
        if (endT == 1) return linear(start, end);
        return heading(Interpolator.piecewise()
                .until(endT, Interpolator.linear(start, end))
                .until(1, Interpolator.constant(end)));
    }

    public Path linear(Pose start, Pose end, double endT) {
        return linear(start.heading(), end.heading(), endT);
    }

    public Path tangent() {
        return heading(Interpolator.tangent);
    }

    public Path reverseTangent() {
        return heading(Interpolator.tangent.reverse());
    }

    public Path facingPoint(Vector2D point) {
        return heading(Interpolator.facingPoint(point));
    }

    public Path facingPoint(Pose pose) {
        return facingPoint(pose.toVector2D());
    }
}
