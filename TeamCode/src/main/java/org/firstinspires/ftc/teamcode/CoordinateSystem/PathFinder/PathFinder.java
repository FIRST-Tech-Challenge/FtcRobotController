package org.firstinspires.ftc.teamcode.CoordinateSystem.PathFinder;

import org.firstinspires.ftc.teamcode.CoordinateSystem.Coordinate;
import org.firstinspires.ftc.teamcode.CoordinateSystem.Path;

public interface PathFinder {
    Path getShortestPath(Coordinate start, Coordinate end);
}
