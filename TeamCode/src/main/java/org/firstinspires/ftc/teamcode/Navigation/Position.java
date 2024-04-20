package org.firstinspires.ftc.teamcode.Navigation;

class Position {
    double x;
    double y;
    double theta;

    Position(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    @Override
    public String toString() {
        return "{" +
                "x=" + x +
                ", y=" + y +
                ", theta=" + theta +
                '}';
    }
}