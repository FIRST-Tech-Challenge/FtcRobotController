package com.kalipsorobotics.math;

import com.kalipsorobotics.PID.PidNav;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;

import org.checkerframework.dataflow.qual.Pure;

import java.util.ArrayList;
import java.util.List;

public class Position {
    final private double x;
    final private double y;
    final private double theta;

    private PidNav pidX = PurePursuitAction.PID_X;
    private PidNav pidY = PurePursuitAction.PID_Y;
    private PidNav pidAngle = PurePursuitAction.PID_ANGLE;

    public Position (double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public Position (double x, double y, double theta, PidNav pidX, PidNav pidY, PidNav pidAngle) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.pidX = pidX;
        this.pidY = pidY;
        this.pidAngle = pidAngle;
    }

    //add point to vector
    public Position add(Velocity velocity) {
        double theta = this.theta + velocity.getTheta();
        return new Position (
            this.x + velocity.getX(),
            this.y + velocity.getY(),
            MathFunctions.angleWrapRad(theta)
        );
    }

    public Position addPosition(Vector vector) {
        return new Position(this.getX() + vector.getX(), this.getY() + vector.getY(), Math.atan2(vector.getY(),vector.getX()));
    }

//    public Point toPoint() {
//        return new Point(getX(), getY());
//    }

//    public static List<Point> toPointList(List<Position> positions) {
//        List<Point> points = new ArrayList<Point>();
//
//        for(int i=0; i < positions.size(); i++) {
//            points.add(positions.get(i).toPoint());
//        }
//
//        return points;
//    }

    public Position relativeTo(Position other) {
        return new Position(this.getX() - other.getX(), this.getY() - other.getY(), this.getTheta() - other.getTheta());
    }

    public Vector projectOnto(Vector vector) {
        return vector.scale( vector.dot(this)/vector.dot(vector));
    }

    public double distanceTo(Position other) {
        return Math.hypot(other.getX() - this.getX(), other.getY() - this.getY());
    }

    @Override
    public String toString() {
        return  String.format("x=%.2f (%.2f in), y=%.2f (%.2f in), theta=%.4f (%.1f deg)", x, x/25.4, y, y/25.4,
                theta, Math.toDegrees(theta));
    }

    public String getPoint() {
        return x + ", " + y + ", " + theta;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return theta;
    }

    public PidNav getPidX() {
        return pidX;
    }

    public PidNav getPidY() {
        return pidY;
    }

    public PidNav getPidAngle() {
        return pidAngle;
    }
}