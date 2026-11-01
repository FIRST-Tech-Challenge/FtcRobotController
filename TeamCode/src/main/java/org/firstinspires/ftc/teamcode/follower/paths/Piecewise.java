/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.paths;

import java.util.*;
import java.util.function.ToDoubleFunction;

public final class Piecewise<T> {
    private final ToDoubleFunction<T> getLength;
    private final double totalLength;
    private final List<Segment<T>> segments = new ArrayList<>();
    private final NavigableMap<Double, Segment<T>> segmentMap = new TreeMap<>();

    public Piecewise(ToDoubleFunction<T> getLength, List<T> items) {
        if (items.isEmpty()) throw new IllegalArgumentException("Piecewise must have at least one item.");

        this.getLength = getLength;
        totalLength = items.stream().mapToDouble(getLength).sum();

        double currentT = 0.0;
        for (T item : items) {
            Segment<T> segment = new Segment<>(currentT, item);
            segments.add(segment);
            segmentMap.put(currentT, segment);
            currentT += getLength.applyAsDouble(item) / totalLength;
        }
    }

    public T get(double t) {
        return getSegment(t).value();
    }

    public Segment<T> getSegment(double t) {
        return segmentMap.floorEntry(t).getValue();
    }

    public double localT(double t) {
        Segment<T> segment = getSegment(t);
        return (t - segment.startT()) / getLength.applyAsDouble(segment.value()) * totalLength;
    }

    public double globalT(Segment<T> segment, double localT) {
        return segment.startT() + localT * getLength.applyAsDouble(segment.value()) / totalLength;
    }

    public double length() {
        return totalLength;
    }

    public List<Segment<T>> segments() {
        return Collections.unmodifiableList(segments);
    }

    public static class Segment<T> {
        private final double startT;
        private final T value;

        public Segment(double startT, T value) {
            this.startT = startT;
            this.value = value;
        }

        public double startT() {
            return startT;
        }

        public T value() {
            return value;
        }
    }
}
