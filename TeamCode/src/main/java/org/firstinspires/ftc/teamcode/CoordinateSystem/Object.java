package org.firstinspires.ftc.teamcode.CoordinateSystem;

import java.util.ArrayList;

public class Object {
    private ArrayList<Coordinate> occupies;

    public Object(ArrayList<Coordinate> occupies) {
        this.occupies = occupies;
    }

    public ArrayList<Coordinate> getOccupies() {
        return this.occupies;
    }
}
