package org.firstinspires.ftc.teamcode.CoordinateSystem;

import androidx.annotation.NonNull;

import java.util.ArrayList;
import java.util.Iterator;

public class Field implements Iterable<int[]>{
    public final int length = 100;
    public final int width = 100;
    private int[][] field = new int[length][width];
    private int x;
    private int y;
    ArrayList<Object> Objects = new ArrayList<Object>();

    public Field() {
        for (x = 0; x < length; x++) {
            for (y = 0; y < width; y++) {
                this.field[x][y] = 0;
            }
        }

    }

    public Field(ArrayList<Object> Objects) {
        for (x = 0; x < length; x++) {
            for (y = 0; y < width; y++) {
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
        if (field[goTo.x][goTo.y] == -1) {
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

    public int get(Coordinate coordinate) {
        return field[coordinate.getX()][coordinate.getY()];
    }

    public void set(Coordinate coordinate, int value) {
        this.field[coordinate.getX()][coordinate.getY()] = value;
    }

    public int[][] getField() {
        return field;
    }

    @NonNull
    @Override
    public Iterator<int[]> iterator() {
        return null;
    }
}
