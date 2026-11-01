/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.paths;

import static org.firstinspires.ftc.teamcode.follower.utils.Utils.listOf;

import org.firstinspires.ftc.teamcode.follower.config.Modifier;
import org.firstinspires.ftc.teamcode.follower.math.Pose;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;

public final class PathTracker {
    private final Deque<PathSegment> segments;
    private final List<Modifier> activeModifiers = new ArrayList<>();
    private final Path path;
    private int currentIndex = 0;

    public PathTracker(Path path) {
        this.path = path;
        List<PathSegment> resolvedSegments = path.getSegments();
        if (resolvedSegments.isEmpty())
            throw new IllegalArgumentException("PathTracker requires at least one path segment");
        segments = new ArrayDeque<>(resolvedSegments);
        PathSegment last = resolvedSegments.get(resolvedSegments.size() - 1);
        endPose = last.curve.get(1).toPose(last.heading(1));
        transitionModifiers(segments.peek().modifiers());
    }

    private final Pose endPose;

    public Pose endPose() {
        return endPose;
    }

    public void advance() {
        if (segments.isEmpty()) throw new IllegalStateException("Cannot advance past last path");
        segments.remove();
        currentIndex++;
        PathSegment current = segments.peek();
        transitionModifiers(current == null ? listOf() : current.modifiers());
    }

    public PathSegment current() {
        return segments.peek();
    }

    public boolean done() {
        return segments.isEmpty();
    }

    public int remainingPaths() {
        return segments.size();
    }

    public void release() {
        transitionModifiers(listOf());
    }

    public int currentIndex() {
        return currentIndex;
    }

    private void transitionModifiers(List<Modifier> nextModifiers) {
        for (int i = activeModifiers.size() - 1; i >= 0; i--) {
            activeModifiers.get(i).revert();
        }

        for (Modifier m : nextModifiers) {
            m.apply();
        }

        activeModifiers.clear();
        activeModifiers.addAll(nextModifiers);
    }

    public Path path() {
        return path;
    }
}
