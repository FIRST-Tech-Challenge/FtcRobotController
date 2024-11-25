package org.firstinspires.ftc.teamcode;


public class FieldPoint2d {
    private final Point2d pointBlue;
    private final Point2d pointRed;

    public FieldPoint2d(Point2d pointBlue) {
        this.pointBlue = pointBlue;
        this.pointRed = new Point2d(-this.pointBlue.x, -this.pointBlue.y);
    }

    public Point2d getPointBlue() {
        return pointBlue;
    }
    public Point2d getPointRed() {
        return pointRed;
    }
}
