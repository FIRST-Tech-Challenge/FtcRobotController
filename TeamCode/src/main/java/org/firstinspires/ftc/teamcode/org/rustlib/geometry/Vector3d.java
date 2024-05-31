package org.firstinspires.ftc.teamcode.org.rustlib.geometry;

import org.firstinspires.ftc.teamcode.org.rustlib.drive.Field;

import java.util.Objects;

public class Vector3d {
    public final double x;
    public final double y;
    public final double z;
    public final double magnitude;
    public final double polar; // See https://en.wikipedia.org/wiki/Spherical_coordinate_system
    public final double azimuthal;

    public Vector3d(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
        magnitude = calculateRadius();
        polar = Vector2d.calculateAngle(Vector2d.calculateRadius(x, y), z);
        azimuthal = Vector2d.calculateAngle(x, y);
    }

    public Vector3d() {
        this(0, 0, 0);
    }

    private double calculateRadius() {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
    }

    public Vector3d rotate(double angleRadians, Axis axis) {
        double[] rotatedPoint;
        switch (axis) {
            case X:
                rotatedPoint = Vector2d.rotate(y, z, angleRadians);
                return new Vector3d(x, rotatedPoint[0], rotatedPoint[1]);
            case Y:
                rotatedPoint = Vector2d.rotate(x, z, angleRadians);
                return new Vector3d(rotatedPoint[0], y, rotatedPoint[1]);
            case Z:
                rotatedPoint = Vector2d.rotate(x, y, angleRadians);
                return new Vector3d(rotatedPoint[0], rotatedPoint[1], z);
        }
        throw new RuntimeException("Could not rotate vector");
    }

    public Vector3d pitch(double angleRadians) {
        return rotate(angleRadians, Axis.Y);
    }

    public Vector3d roll(double angleRadians) {
        return rotate(angleRadians, Axis.X);
    }

    public Vector3d yaw(double angleRadians) {
        return rotate(angleRadians, Axis.Z);
    }

    public Vector3d translateX(double x) {
        return translate(x, 0, 0);
    }

    public Vector3d translateY(double y) {
        return translate(0, y, 0);
    }

    public Vector3d translateZ(double z) {
        return translate(0, 0, z);
    }

    public Vector3d translate(double x, double y, double z) {
        return new Vector3d(this.x + x, this.y + y, this.z + z);
    }

    public Vector3d mirror() {
        return new Vector3d(x, Field.fieldLengthIn - y, z);
    }

    @Override
    public boolean equals(Object vector) {
        if (vector instanceof Vector3d) {
            Vector3d toCompare = (Vector3d) vector;
            if (!(Double.isFinite(toCompare.x) && Double.isFinite(toCompare.y) && Double.isFinite(toCompare.z)) && !(Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(z))) {
                return true;
            }
            return Objects.equals(x, toCompare.x) && Objects.equals(y, toCompare.y);
        }
        return false;
    }

    public enum Axis {
        X,
        Y,
        Z
    }
}
