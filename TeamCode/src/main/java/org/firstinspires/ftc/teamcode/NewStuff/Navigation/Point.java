package org.firstinspires.ftc.teamcode.NewStuff.Navigation;

import androidx.annotation.NonNull;

public class Point {

    final private double x;
    final private double y;

    public Point (double x, double y) {
        this.x = x;
        this.y = y;
    }

    @Override
    public String toString() {
        return "Point{" +
                "x=" + x +
                ", y=" + y +
                '}';
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

}
