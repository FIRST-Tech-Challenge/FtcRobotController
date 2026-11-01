/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.follower.api;

import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.follower.utils.Angle;
import java.util.function.DoubleUnaryOperator;

public final class PoseFactory {
    private final Operation operation;
    private final Angle.Unit angleUnit;

    public PoseFactory(Operation operation, Angle.Unit angleUnit) {
        this.operation = operation;
        this.angleUnit = angleUnit;
    }

    public PoseFactory(Operation operation, boolean useDegrees) {
        this(operation, useDegrees ? Angle.Unit.DEGREES : Angle.Unit.RADIANS);
    }

    /**
     * Creates a PoseFactory that uses degrees for heading.
     */
    public static PoseFactory degrees() {
        return new PoseFactory(Operation.IDENTITY, Angle.Unit.DEGREES);
    }

    /**
     * Creates a PoseFactory that uses radians for heading.
     */
    public static PoseFactory radians() {
        return new PoseFactory(Operation.IDENTITY, Angle.Unit.RADIANS);
    }

    /**
     * Creates a Pose with the given x, y, and heading.
     * The heading is interpreted in the unit specified by the PoseFactory (degrees or radians).
     * Any operations defined in the PoseFactory will be applied to the created Pose.
     */
    public Pose of(double x, double y, double heading) {
        return operation.apply(new Pose(x, y, angleUnit.toRadians(heading)));
    }

    public PoseFactory map(Operation operator) {
        return new PoseFactory(operation.andThen(operator), angleUnit);
    }

    /**
     * Returns a new PoseFactory that reflects the Pose across the vertical line at the specified
     * x-coordinate, mirroring the x-coordinate and the heading.
     */
    public PoseFactory mirrorX(double axis) {
        return map(pose -> new Pose(2 * axis - pose.x(), pose.y(), Math.PI - pose.heading()));
    }

    /**
     * Returns a new PoseFactory that reflects the Pose across the horizontal line at the specified
     * y-coordinate, mirroring the y-coordinate and the heading.
     */
    public PoseFactory mirrorY(double axis) {
        return map(pose -> new Pose(pose.x(), 2 * axis - pose.y(), -pose.heading()));
    }

    public PoseFactory mirrorAroundPoint(double centerX, double centerY) {
        return rotateAround(centerX, centerY, angleUnit.fromRadians(Math.PI));
    }

    public PoseFactory mirrorAroundPoint(Pose center) {
        return mirrorAroundPoint(center.x(), center.y());
    }

    /**
     * Returns a new PoseFactory that rotates the Pose counter-clockwise around the specified point
     * by the specified angle, which is interpreted in the unit specified by the PoseFactory
     * (degrees or radians).
     */
    public PoseFactory rotateAround(double centerX, double centerY, double angle) {
        double radians = angleUnit.toRadians(angle);
        double cos = Math.cos(radians);
        double sin = Math.sin(radians);
        return map(pose -> {
            double dx = pose.x() - centerX;
            double dy = pose.y() - centerY;
            return new Pose(centerX + dx * cos - dy * sin, centerY + dx * sin + dy * cos, pose.heading() + radians);
        });
    }

    /**
     * Returns a new PoseFactory that rotates the Pose counter-clockwise around the specified point
     * by the specified angle, which is interpreted in the unit specified by the PoseFactory
     * (degrees or radians). Only the point's coordinates are used; its heading is ignored.
     */
    public PoseFactory rotateAround(Pose point, double angle) {
        return rotateAround(point.x(), point.y(), angle);
    }

    public PoseFactory mapX(DoubleUnaryOperator operator) {
        return map(pose -> pose.withX(operator.applyAsDouble(pose.x())));
    }

    public PoseFactory mapY(DoubleUnaryOperator operator) {
        return map(pose -> pose.withY(operator.applyAsDouble(pose.y())));
    }

    public PoseFactory mapHeading(DoubleUnaryOperator operator) {
        return map(pose ->
                pose.withHeading(angleUnit.toRadians(operator.applyAsDouble(angleUnit.fromRadians(pose.heading())))));
    }

    @FunctionalInterface
    public interface Operation {
        Operation IDENTITY = pose -> pose;

        Pose apply(Pose pose);

        default Operation andThen(Operation operator) {
            return pose -> operator.apply(apply(pose));
        }
    }
}
