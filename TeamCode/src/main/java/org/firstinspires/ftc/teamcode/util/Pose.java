package org.firstinspires.ftc.teamcode.util;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Vector2D;

import java.util.Locale;

public class Pose extends Point {

    public double heading;

    public Pose(double x, double y, double heading) {
        super(x, y);
        this.heading = heading;
    }
    public Pose(Point p, double heading) {
        this(p.x, p.y, heading);
    }

    public Pose(Vector2D vec, double heading) {
        this(vec.x, vec.y, heading);
    }

    public Pose(){
        this(0, 0, 0);
    }

    public Pose add(Pose other) {
        return new Pose(x + other.x, y + other.y, heading + other.heading);
    }

    public Pose subtract(Pose other) {
        return new Pose(this.x - other.x, this.y - other.y, AngleUnit.normalizeRadians(this.heading - other.heading));
    }

    public Pose divide(Pose other) {
        return new Pose(this.x / other.x, this.y / other.y, this.heading / other.heading);
    }

    public Pose subt(Pose other) {
        return new Pose(x - other.x, y - other.y, heading - other.heading);
    }

    public Vector2D toVec2D() {
        return new Vector2D(x, y);
    }

    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "%.2f %.2f %.2f", x, y, heading);
    }
}