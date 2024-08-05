package org.firstinspires.ftc.teamcode.NewStuff.Math;

import java.util.Optional;

public class Segment {

    private final Point start;
    private final Point finish;

    public Segment(Point start, Point finish) {
        this.start = start;
        this.finish = finish;
    }

    public Optional<Point> lineCircleIntersection(Point current, double radius) {
        Point shiftedCurrent = current.relativeTo(this.getStart());
        Optional<Vector> shiftedFollow = lineCircleIntersection(this.getVector(), shiftedCurrent, radius);

        return shiftedFollow.map(this.getStart()::add);
    }
    private static Optional<Vector> lineCircleIntersection(Vector vector, Point shiftedCurrent, double radius) {
        Vector projection = shiftedCurrent.projectOnto(vector);
        double distance = Math.hypot(shiftedCurrent.getX() - projection.getX(), shiftedCurrent.getY() - projection.getY());

        Vector follow;
        if (distance > radius) {
            // edge case, handle no intersection
            return Optional.empty();
        } else {
            follow =
                    projection.add(vector.withLength(Math.sqrt(Math.pow(radius, 2) - Math.pow(distance, 2))));
        }
        // edge case, handle ||f|| > ||v|| (past the point)
        if (follow.getLength() > vector.getLength()) {
            return Optional.of(vector);
        }
        // edge case, handle f behind v, then no intersection
        if (vector.dot(follow) < 0) {
            return Optional.empty();
        }
        return Optional.of(follow);
    }

    public Vector getVector() {
        return Vector.between(this.start, this.finish);
    }

    public Point getStart() {
        return start;
    }

    public Point getFinish() {
        return finish;
    }

    public double getHeadingDirection() {
        return this.getVector().getHeadingDirection();
    }
}
