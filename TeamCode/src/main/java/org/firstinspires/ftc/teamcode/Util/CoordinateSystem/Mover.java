package org.firstinspires.ftc.teamcode.Util.CoordinateSystem;

import java.util.ArrayList;

public class Mover extends Object {
    private Coordinate center;

    public Mover(ArrayList<Coordinate> occupies, Coordinate center) {
        super(occupies);
        this.center = center;
    }
}
