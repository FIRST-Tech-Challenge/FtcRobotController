package org.firstinspires.ftc.teamcode.Util.CoordinateSystem.PathFinder;

import org.firstinspires.ftc.teamcode.Util.CoordinateSystem.Coordinate;
import org.firstinspires.ftc.teamcode.Util.CoordinateSystem.Path;

public interface PathFinder {
    Path getShortestPath(Coordinate start, Coordinate end);
}
