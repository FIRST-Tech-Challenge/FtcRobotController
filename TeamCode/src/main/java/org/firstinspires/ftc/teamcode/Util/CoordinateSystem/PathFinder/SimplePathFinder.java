package org.firstinspires.ftc.teamcode.Util.CoordinateSystem.PathFinder;

import org.firstinspires.ftc.teamcode.Util.CoordinateSystem.Coordinate;
import org.firstinspires.ftc.teamcode.Util.CoordinateSystem.Path;

public class SimplePathFinder implements PathFinder {
    @Override
    public Path getShortestPath(Coordinate start, Coordinate end) {
        Path path = new Path();
        int xMove = end.getX() - start.getX();
        int yMove = end.getY() - start.getY();

        int xMoveNow;
        int yMoveNow;
        Coordinate current = start;
        while (xMove != 0 && yMove != 0) {
            if (xMove > 0) {
                xMoveNow = 1;
                xMove--;
            } else if (xMove < 0) {
                xMoveNow = -1;
                xMove++;
            } else {
                xMoveNow = 0;
            }

            if (yMove > 0) {
                yMoveNow = 1;
                yMove--;
            } else if (yMove < 0) {
                yMoveNow = -1;
                yMove++;
            } else {
                yMoveNow = 0;
            }

            current = new Coordinate(current.getX() + xMoveNow, current.getY() + yMoveNow);
            path.add(current);
        }
        return path;
    }
}
