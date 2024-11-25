package com.kalipsorobotics.math;

import java.util.Collections;
import java.util.List;
import java.util.Optional;

public class Path {
    private final List<Position> path;
    private int currentSearchWayPointIndex = 0;

    public Path(List<Position> path) {
        this.path = Collections.unmodifiableList(path);
    }

//    public Optional<Position> searchFrom(Position currentPosition, double radius) {
//        for (int i = numSegments() - 1; i >= 0; i--) {
//            Segment segment = getSegment(i);
//
//            Optional<Position> result = segment.lineCircleIntersection(currentPosition, radius);
//
//            if (result.isPresent()) {
//                return result;
//            }
//        }
//
//        return Optional.empty();
//    }

    public Optional<Position> searchFrom(Position currentPosition, double radius) {
        for (int i = numSegments() - 1; i >= 0; i--) {
            Segment segment = getSegment(i);

            Optional<Position> result = segment.lineCircleIntersection(currentPosition, radius);

            if (result.isPresent()) {
                Position position = new Position(result.get().getX(), result.get().getY(), segment.getFinish().getTheta());
                return Optional.of(position);
            }
        }

        return Optional.empty();
    }

    public Optional<Position> lookAhead(Position currentPosition, double radiusInch) {
        for(int i = currentSearchWayPointIndex; i<numPoints(); i++) {
            if (currentPosition.distanceTo(getPoint(i)) > radiusInch) {
                Position position = getPoint(i);
                currentSearchWayPointIndex = i;
                return Optional.of(position);
            }
        }

        if(Math.abs(currentPosition.getTheta() - getPoint(currentSearchWayPointIndex).getTheta()) < Math.toRadians(4)) {
            return Optional.empty();
        } else {
            return Optional.of(getPoint(currentSearchWayPointIndex));
        }

    }

    public Position getPoint(int index) {
        return path.get(index);
    }

    public int numPoints() {
        return path.size();
    }

    public int numSegments() {
        return numPoints() - 1;
    }

    public Segment getSegment(int index) {
        return new Segment(getPoint(index), getPoint(index + 1));
    }
}
