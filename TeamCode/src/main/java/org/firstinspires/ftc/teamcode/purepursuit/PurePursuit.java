package org.firstinspires.ftc.teamcode.purepursuit;

import java.util.ArrayList;
import java.util.List;

public class PurePursuit {
    List<LineSegment> path;
    public final double LOOKAHEAD_RADIUS = 1.0;
    public double robotX = 0, robotY = 0;
    public ArrayList<Double> intersections;
    public int lastIntersectionIndex = -1;
    public PurePursuit(double robotX, double robotY) {
        this.robotX = robotX;
        this.robotY = robotY;
        this.path = new ArrayList<>();
        this.intersections = new ArrayList<>();
    }
    public void updatePositions(double velocity, double angle, double timeslice) {
        this.robotX += velocity * Math.cos(angle) * timeslice;
        this.robotY += velocity * Math.sin(angle) * timeslice;
    }

    public boolean findIntersections(int index) {
        LineSegment lineSegment = this.path.get(index);
        double x0 = lineSegment.x0, y0 = lineSegment.y0, x1 = lineSegment.x1, y1 = lineSegment.y1;
        double a = Math.pow(x1 - x0, 2) + Math.pow(y1 - y0, 2);
        double b = 2 * ((x0 - this.robotX) * (x1 - x0) + (y0 - this.robotY) * (y1 - y0));
        double c = Math.pow(x0 - this.robotX, 2)
                + Math.pow(y0 - this.robotY, 2)
                - Math.pow(this.LOOKAHEAD_RADIUS, 2);
//        double c = Math.pow(x0, 2) - 2 * this.robotX * x0 + Math.pow(this.robotX, 2)
//                + Math.pow(y0, 2) - 2 * this.robotY * y0 + Math.pow(this.robotY, 2)
//                - Math.pow(this.LOOKAHEAD_RADIUS, 2);
        double discriminant = Math.pow(b, 2) - 4 * a * c;
        if (discriminant < 0) return false;
//        double t1 = (-b - Math.sqrt(discriminant)) / (2 * a);
        double t2 = (-b + Math.sqrt(discriminant)) / (2 * a);
        if (t2 < 0 || t2 > 1) return false;
        intersections.add(t2);
        this.lastIntersectionIndex = index;
        return true;
    }

    public double calculateMovementAngle() {
        double tLast = intersections.get(intersections.size() - 1);
        LineSegment segmentLast = path.get(this.lastIntersectionIndex);
        double targetx = segmentLast.x0 + (segmentLast.x1 - segmentLast.x0) * tLast;
        double targety = segmentLast.y0 + (segmentLast.y1 - segmentLast.y0) * tLast;

        double movementAngle = Math.atan((targety - robotY) / (targetx - robotX));


    }

    public void addSegmentToPath(LineSegment segment) {
        path.add(segment);
    }

    public void run(String[] args) {
        // This is the path that the robot will follow
        // 1.0 --> 1 wheel rotation
        this.addSegmentToPath(new LineSegment(1.0, 2.0, 3.0, 4.0));
        this.addSegmentToPath(new LineSegment(3.0, 4.0, 5.0, 6.0));
        this.addSegmentToPath(new LineSegment(1.0, 2.0, 3.0, 4.0));

    }
}
