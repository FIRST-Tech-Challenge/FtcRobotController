package com.kalipsorobotics.math;

import com.kalipsorobotics.PID.PidNav;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;

public class Position {
    private double x;
    private double y;
    private double theta;

    private PidNav pidX = new PidNav(PurePursuitAction.P_XY, 0, 0);
    private PidNav pidY = new PidNav(PurePursuitAction.P_XY, 0, 0);
    private PidNav pidAngle = new PidNav(PurePursuitAction.P_ANGLE, 0, 0);

    public Position (double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public Position (Position position) {
        this(position.x, position.y, position.theta);
    }

    public Position (double x, double y, double theta, double pXY, double pAngle) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.pidX.setP(pXY);
        this.pidY.setP(pXY);
        this.pidAngle.setP(pAngle);
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

    public void reset(Position position) {
        this.x = position.getX();
        this.y = position.getY();
        this.theta = position.getTheta();
        this.pidX = position.pidX;
        this.pidY = position.pidY;
        this.pidAngle = position.pidAngle;
    }
}