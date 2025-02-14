package com.kalipsorobotics.math;

import java.util.Collections;
import java.util.List;
import java.util.Optional;

public class Path {
    private final List<Position> path;
    private int currentSearchWayPointIndex = 0;

    private final double PATH_ANGLE_TOLERANCE = Math.toRadians(10);

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

    public Optional<Position> lookAhead(Position currentPosition, Optional<Position> lastFollowPosition, double radiusInch) {
        for (int i = currentSearchWayPointIndex; i < numPoints(); i++) {
            Position currentFollowPosition = getPoint(i);

            if (!lastFollowPosition.isPresent()) {
                currentSearchWayPointIndex = i;
                return Optional.of(currentFollowPosition);
            }

            if (currentPosition.distanceTo(currentFollowPosition) > radiusInch) {
                currentSearchWayPointIndex = i;
                return Optional.of(currentFollowPosition);
            }/* else {
                if (Math.abs(currentPosition.getTheta() - currentFollowPosition.getTheta()) > PATH_ANGLE_TOLERANCE) {
                    currentSearchWayPointIndex = i;
                    return Optional.of(currentFollowPosition);
                }
            }*/
        }
        return Optional.empty();
    }

    public Position getPoint(int index) {
        return path.get(index);
    }

    public int numPoints() {
        return path.size();
    }

    public Position getLastPoint() {
        return path.get(path.size() - 1);
    }

    public int numSegments() {
        return numPoints() - 1;
    }

    public Segment getSegment(int index) {
        return new Segment(getPoint(index), getPoint(index + 1));
    }

    public int findIndex(Position position) {
        for (int i = 0; i < path.size(); i++) {

            if (position == path.get(i)) {
                return i;
            }

        }
        return -1;
    }
}
