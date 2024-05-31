package org.firstinspires.ftc.teamcode.org.rustlib.drive;

import org.firstinspires.ftc.teamcode.org.rustlib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.org.rustlib.geometry.Vector2d;

import java.util.function.Supplier;

public class Waypoint extends Vector2d implements Supplier<Waypoint> {
    public final double followRadius;

    public final Rotation2d targetFollowRotation;

    public final Rotation2d targetEndRotation;

    public final double maxVelocity;

    public Waypoint(Vector2d vector, double followRadius, Rotation2d targetFollowRotation, Rotation2d targetEndRotation, double maxVelocity) {
        super(vector.x, vector.y);
        this.targetFollowRotation = targetFollowRotation;
        this.targetEndRotation = targetEndRotation;
        this.followRadius = followRadius;
        this.maxVelocity = Math.abs(maxVelocity);
    }

    public Waypoint(Vector2d vector, double followRadius) {
        this(vector, followRadius, null, null, Double.POSITIVE_INFINITY);
    }

    public Waypoint(double x, double y, double followRadius, Rotation2d targetFollowRotation, Rotation2d targetEndRotation, double maxVelocity) {
        this(new Vector2d(x, y), followRadius, targetFollowRotation, targetEndRotation, maxVelocity);
    }

    public Waypoint(double x, double y, double followRadius, Rotation2d targetEndRotation, double maxVelocity) {
        this(new Vector2d(x, y), followRadius, null, targetEndRotation, maxVelocity);
    }

    public Waypoint(double x, double y, double followRadius, Rotation2d targetFollowRotation, Rotation2d targetEndRotation) {
        this(x, y, followRadius, targetFollowRotation, targetEndRotation, Double.POSITIVE_INFINITY);
    }

    public Waypoint(double x, double y, double followRadius, double maxVelocity) {
        this(x, y, followRadius, null, null, maxVelocity);
    }

    public Waypoint(double x, double y, double followRadius) {
        this(x, y, followRadius, null, null, Double.POSITIVE_INFINITY);
    }

    public Waypoint translateX(double x) {
        return translate(x, 0);
    }

    public Waypoint translateY(double y) {
        return translate(0, y);
    }

    public Waypoint translate(double x, double y) {
        return new Waypoint(x + x, y + y, followRadius, targetFollowRotation, targetEndRotation, maxVelocity);
    }

    public Waypoint rotate(Rotation2d rotation) {
        return new Waypoint(x, y, followRadius, targetFollowRotation.add(rotation), targetEndRotation.add(rotation), maxVelocity);
    }

    public Waypoint mirror() {
        return new Waypoint(x, Field.fieldLengthIn - y, followRadius, targetFollowRotation.negate().addRadians(Math.PI), targetEndRotation.negate().addRadians(Math.PI), maxVelocity);
    }

    @Override
    public String toString() {
        return "x: " + x + " y: " + y + " follow radius: " + followRadius + " target follow rot: " + Rotation2d.getString(targetFollowRotation) + " target end rot: " + Rotation2d.getString(targetEndRotation) + " max vel: " + maxVelocity;
    }

    @Override
    public Waypoint get() {
        return this;
    }
}
