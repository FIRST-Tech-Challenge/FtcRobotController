package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.drivecontrol.Angle;
import org.firstinspires.ftc.teamcode.drivecontrol.Vector2d;

public class Vector2D {
    public final static Vector2D
            FORWARD = new Vector2D(0, 1),
            BACKWARD = new Vector2D(0, -1),
            LEFT = new Vector2D(-1, 0),
            RIGHT = new Vector2D(1, 0),
            ZERO = new Vector2D(0, 0),
            UNIT_CIRCLE_60 = new Vector2D(.5, Math.sqrt(3)/2),
            UNIT_CIRCLE_120 = new Vector2D(-0.5, Math.sqrt(3)/2);

    private double x;
    private double y;

    public final static int CW = 0;
    public final static int CCW = 1;

    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getAngle() {
        return Math.atan2(y, x);
    }

    public double getMagnitude() {
        return Math.sqrt(x * x + y * y);
    }

    public Vector2D add(Vector2D other) {
        return new Vector2D(x + other.getX(), y + other.getY());
    }

    public Vector2D scale(double scale) {
        return new Vector2D(getX() * scale, getY() * scale);
    }

    public Vector2D normalize(double target) {
        if (getMagnitude() == 0) return ZERO; //avoid dividing by zero
        return scale(target / getMagnitude());
    }

    public static Vector2D[] batchNormalize(double limit, Vector2D... vecs) {
        double maxMag = 0;
        for (Vector2D v : vecs) {
            if (v.getMagnitude() > maxMag) {
                maxMag = v.getMagnitude();
            }
        }
        if (limit >= maxMag) {
            return vecs;
        }
        Vector2D[] normed = new Vector2D[vecs.length];
        for (int i = 0; i < vecs.length; i++) {
            normed[i] = vecs[i].scale(limit / maxMag);
        }
        return normed;
    }

    public Vector2D rotateBy(double ang, int direction) {
        double angRads;
        if (direction == Vector2D.CCW) {
            angRads = Math.toRadians(ang); //default vector rotation direction is CCW
        } else {
            angRads = -1 * Math.toRadians(ang);
        }
        return new Vector2D(x * Math.cos(angRads) - y * Math.sin(angRads), x * Math.sin(angRads) + y * Math.cos(angRads));
    }

    public Vector2D reflect () {
        return new Vector2D(-x, -y);
    }

    public double dot(Vector2D other) {
        return getX() * other.getX() + getY() * other.getY();
    }

    public Vector2D projection (Vector2D v) {
        return v.scale(dot(v)/(Math.pow(v.getMagnitude(), 2))); // u dot v over mag(v)^2 times v
    }
}
