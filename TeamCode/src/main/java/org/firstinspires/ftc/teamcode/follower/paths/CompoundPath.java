/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.paths;

import static org.firstinspires.ftc.teamcode.follower.utils.Utils.*;

import org.firstinspires.ftc.teamcode.follower.config.Modifier;
import org.firstinspires.ftc.teamcode.follower.paths.curves.CompoundCurve;
import org.firstinspires.ftc.teamcode.follower.paths.curves.Curve;
import org.firstinspires.ftc.teamcode.follower.paths.interpolator.Interpolator;
import java.util.List;

public class CompoundPath extends Path {
    private final Interpolator interpolator;
    private final Piecewise<Path> paths;

    private CompoundPath(Curve curve, Interpolator interpolator, List<Modifier> modifiers, Piecewise<Path> paths) {
        super(curve, modifiers);
        this.interpolator = interpolator;
        this.paths = paths;
    }

    public CompoundPath(Interpolator interpolator, List<Modifier> modifiers, List<Path> paths) {
        super(new CompoundCurve(paths.stream().map(path -> path.curve).collect(toUnmodifiableList())), modifiers);
        this.interpolator = interpolator;
        this.paths = new Piecewise<>(path -> path.curve.length(), paths);
    }

    public CompoundPath(Path... paths) {
        this(null, listOf(), listOf(paths));
    }

    @Override
    public double heading(double t) {
        TValue.check(t);
        if (interpolator != null) {
            if (!interpolator.isGeometric()) return interpolator.interpolate(curve, curve.pathCompletion(t));
            return interpolator.interpolate(curve, t);
        }
        return paths.get(t).heading(paths.localT(t));
    }

    @Override
    public boolean hasHeading() {
        if (interpolator != null) return true;
        return paths.segments().stream().map(Piecewise.Segment::value).allMatch(Path::hasHeading);
    }

    public List<Piecewise.Segment<Path>> segments() {
        return paths.segments();
    }

    @Override
    protected List<PathSegment> getSegments(PathSegment.HeadingProvider parentHeading, List<Modifier> modifiers) {
        return paths.segments().stream()
                .flatMap(segment -> segment
                        .value()
                        .getSegments(convertChildHeading(parentHeading, segment), concat(modifiers, this.modifiers))
                        .stream())
                .collect(toUnmodifiableList());
    }

    private PathSegment.HeadingProvider convertChildHeading(
            PathSegment.HeadingProvider parentHeading, Piecewise.Segment<Path> segment) {
        double totalLength = curve.length();
        double childLength = segment.value().curve.length();
        double segStart = segment.startT();
        double ratio = childLength / totalLength;
        return t -> parentHeading.heading(segStart + t * ratio);
    }

    @Override
    protected Path withHeading(Interpolator interpolator) {
        return new CompoundPath(curve, interpolator, modifiers, paths);
    }

    @Override
    protected Path withModifiers(List<Modifier> modifiers) {
        return new CompoundPath(curve, interpolator, modifiers, paths);
    }
}
