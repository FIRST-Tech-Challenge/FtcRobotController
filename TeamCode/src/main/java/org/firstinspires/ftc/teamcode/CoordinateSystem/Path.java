package org.firstinspires.ftc.teamcode.CoordinateSystem;

import java.util.ArrayList;

public class Path {
    ArrayList<Coordinate> path;

    public Path () {
        path = new ArrayList<Coordinate>();
    }

    public Path (ArrayList<Coordinate> path) {
        this.path = path;
    }

    public void addCoordinate (Coordinate c) {
        path.add(c);
    }

    public Coordinate getCoordinate (int index) {
        return path.get(index);
    }

    public int getLength () {
        return path.size();
    }
}
