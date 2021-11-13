package org.firstinspires.ftc.teamcode.Util.CoordinateSystem;

import org.firstinspires.ftc.teamcode.Util.CoordinateSystem.PathFinder.SimplePathFinder;

import java.util.ArrayList;

/**
 * A pathfinder demo
 * Uses the Freight Frenzy board
 */
public class Demo {
    public Demo() {
        Field field = new Field();
        Path path = new Path();
        ArrayList<Coordinate> allianceHubCoordinates = new ArrayList<>();
        allianceHubCoordinates.add(new Coordinate(10, 10));
        allianceHubCoordinates.add(new Coordinate(9, 10));
        allianceHubCoordinates.add(new Coordinate(11, 10));
        allianceHubCoordinates.add(new Coordinate(10, 11));
        allianceHubCoordinates.add(new Coordinate(10, 9));

        Object allianceHub = new Object(allianceHubCoordinates);
        field.addObject(allianceHub);

        Coordinate start = new Coordinate(0, 0);
        path.add(start);
        SimplePathFinder pF = new SimplePathFinder();
        path.goTo(allianceHub, pF);
    }
}
