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

    public void add (Coordinate c) {
        path.add(c);
    }

    public void add (int index, Coordinate c) {
        path.add(index, c);
    }

    public Coordinate get (int index) {
        return path.get(index);
    }

    public int length () {
        return path.size();
    }
}
