package org.firstinspires.ftc.teamcode.NewStuff.Math;

import java.util.Collections;
import java.util.List;
import java.util.Optional;

public class Path {
    private final List<Point> path;

    public Path(List<Point> path) {
        this.path = Collections.unmodifiableList(path);
    }

    public Optional<Point> searchFrom(Point currentPosition, double radius) {
        for (int i = numSegments() - 1; i >= 0; i--) {
            Segment segment = getSegment(i);

            Optional<Point> result = segment.lineCircleIntersection(currentPosition, radius);

            if (result.isPresent()) {
                return result;
            }
        }

        return Optional.empty();
    }

    public Point getPoint(int index) {
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
