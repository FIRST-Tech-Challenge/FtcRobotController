package org.firstinspires.ftc.teamcode.rework.Robot.Auto.PathPlanning;

/**
 * Used to define a point in space. Has two field variables, x and y to define a point (x,y)
 * on an euclidean plane. Empty constructor creates a point with value (0, 0).
 */
public class Point {
    public double x;
    public double y;

    Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Creates a point with value (0,0)
     */
    Point() {
        this.x = 0;
        this.y = 0;
    }
}
