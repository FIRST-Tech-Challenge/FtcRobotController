package org.firstinspires.ftc.teamcode.CoordinateSystem;

import java.util.ArrayList;

public class Field {
    private final int size = 1000;
    private int[][] field = new int[size][size];
    private int x;
    private int y;
    ArrayList<Object> Objects = new ArrayList<Object>();

    public Field() {
        for (x = 0; x < size; x++) {
            for (y = 0; y < size; y++) {
                this.field[x][y] = 0;
            }
        }

    }

    public Field(ArrayList<Object> Objects) {
        for (x = 0; x < size; x++) {
            for (y = 0; y < size; y++) {
                this.field[x][y] = 0;
            }
        }
        this.Objects = Objects;
        for (Object object:Objects) {
            for (Coordinate coordinate:object.getOccupies()) {
                this.field[coordinate.x][coordinate.y] = 1;
            }
        }
    }

    public boolean isBlocked(Coordinate goTo) {
        if (field[goTo.x][goTo.y] != 0) {
            return true;
        }
        return false;
    }

    public void addObject(Object object) {
        for (Coordinate coordinate:object.getOccupies()) {
            this.field[coordinate.x][coordinate.y] = 1;
        }
        Objects.add(object);
    }
}
