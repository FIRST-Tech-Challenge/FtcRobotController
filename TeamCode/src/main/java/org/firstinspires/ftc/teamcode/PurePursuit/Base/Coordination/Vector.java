package org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination;

public class Vector {

    public volatile double x, y;

    public Vector (double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector (Vector vector) {
        this.x = vector.x;
        this.y = vector.y;
    }
}
