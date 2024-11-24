package com.kalipsorobotics.math;

import java.util.Optional;

public class Segment {

    private final Position start;
    private final Position finish;

    public Segment(Position start, Position finish) {
        this.start = start;
        this.finish = finish;
    }

    public Optional<Position> lineCircleIntersection(Position current, double radius) {
        Position shiftedCurrent = current.relativeTo(this.getStart());
        Optional<Vector> shiftedFollow = lineCircleIntersection(this.getVector(), shiftedCurrent, radius);

        return shiftedFollow.map(this.getStart()::addPosition);
    }

    private static Optional<Vector> lineCircleIntersection(Vector vector, Position shiftedCurrent, double radius) {
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

    public Position getStart() {
        return start;
    }

    public Position getFinish() {
        return finish;
    }

    public double getHeadingDirection() {
        return this.getVector().getHeadingDirection();
    }
}
