package org.firstinspires.ftc.teamcode;

//credit: this class is based on code from FRC 5818 (https://github.com/Team5818/DiffSwerve)

public class Vector2d {

    //Vector constants
    final static Vector2d FORWARD = new Vector2d(0, 1),
            BACKWARD = new Vector2d(0, -1),
            LEFT = new Vector2d(-1, 0),
            RIGHT = new Vector2d(1, 0),
            ZERO = new Vector2d(0, 0);

    private double x;
    private double y;
    
    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    //makes a unit vector with a certain angle
    public Vector2d(Angle angle) {
        this.x = Math.cos(Math.toRadians(angle.convertAngle(Angle.AngleType.NEG_180_TO_180_CARTESIAN).getAngle()));
        this.y = Math.sin(Math.toRadians(angle.convertAngle(Angle.AngleType.NEG_180_TO_180_CARTESIAN).getAngle()));
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public void setX (double x) { this.x = x; }

    public void setY (double y) { this.y = y; }

    public double getMagnitude() {
        return Math.sqrt(x * x + y * y);
    }

    //returns Angle object
    public Angle getAngle() {
        double angRad = Math.atan2(y, x);
        return new Angle(Math.toDegrees(angRad), Angle.AngleType.NEG_180_TO_180_CARTESIAN);
    }

    public Vector2d add(Vector2d other) {
        return new Vector2d(x + other.getX(), y + other.getY());
    }

    public Vector2d scale(double scale) {
        return new Vector2d(getX() * scale, getY() * scale);
    }

    public Vector2d getUnitVector() {
        return normalize(1);
    }

    //returns a Vector2d in the same direction with magnitude of "target"
    public Vector2d normalize(double target) {
        return scale(target / getMagnitude());
    }

    //returns Vector2d rotated by ang degrees
    public Vector2d rotateBy(double ang, Angle.Direction direction) {
        double angRads;
        if (direction == Angle.Direction.COUNTER_CLOCKWISE) {
            angRads = Math.toRadians(ang); //default vector rotation direction is CCW
        } else {
            angRads = -1 * Math.toRadians(ang);
        }
        return new Vector2d(x * Math.cos(angRads) - y * Math.sin(angRads), x * Math.sin(angRads) + y * Math.cos(angRads));
    }

    //returns Vector2d with the same magnitude as this but at the same angle as an Angle object
    public Vector2d rotateTo (Angle ang) {
        return new Vector2d(ang).scale(this.getMagnitude());
    }

    //dot product
    public double dot(Vector2d other) {
        return getX() * other.getX() + getY() * other.getY();
    }

    //returns Vector2d reflected into 1st quadrant
    public Vector2d abs() {
        return new Vector2d(Math.abs(x), Math.abs(y));
    }

    //flips the signs of both components
    public Vector2d reflect () {
        return new Vector2d(-x, -y);
    }

    //projection of current vector onto v
    public Vector2d projection (Vector2d v) {
        return v.scale(dot(v)/(Math.pow(v.getMagnitude(), 2))); // u dot v over mag(v)^2 times v
    }

    //normalizes a group of vectors so that they maintain the same relative magnitudes and ...
    // the vector of largest magnitude now has a magnitude equal to limit
    public static Vector2d[] batchNormalize(double limit, Vector2d... vecs) {
        double maxMag = 0;
        for (Vector2d v : vecs) {
            if (v.getMagnitude() > maxMag) {
                maxMag = v.getMagnitude();
            }
        }
        if (limit >= maxMag) {
            return vecs;
        }
        Vector2d[] normed = new Vector2d[vecs.length];
        for (int i = 0; i < vecs.length; i++) {
            normed[i] = vecs[i].scale(limit / maxMag);
        }
        return normed;
    }


    public Vector2d clone() {
        return new Vector2d(x,y);
    }

    @Override
    public String toString() {
        return String.format("(%s, %s)", x, y);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof Vector2d)) {
            return false;
        }
        Vector2d other = (Vector2d) obj;
        if (Double.doubleToLongBits(x) != Double.doubleToLongBits(other.x)) {
            return false;
        }
        if (Double.doubleToLongBits(y) != Double.doubleToLongBits(other.y)) {
            return false;
        }
        return true;
    }
}