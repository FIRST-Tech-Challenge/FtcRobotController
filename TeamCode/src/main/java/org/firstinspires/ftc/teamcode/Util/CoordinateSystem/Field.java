package org.firstinspires.ftc.teamcode.Util.CoordinateSystem;

import androidx.annotation.NonNull;

import java.util.ArrayList;
import java.util.Iterator;

public class Field implements Iterable<int[]> {
    public final int length = 100;
    public final int width = 100;
    ArrayList<Object> Objects = new ArrayList<Object>();
    private int[][] field = new int[length][width];
    private int x;
    private int y;

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
        for (Object object : Objects) {
            for (Coordinate coordinate : object.getOccupies()) {
                this.field[coordinate.getX()][coordinate.getY()] = 1;
            }
        }
    }

    public boolean isBlocked(Coordinate goTo) {
        if (field[goTo.getX()][goTo.getY()] == -1) {
            return true;
        }
        return false;
    }

    public void addObject(Object object) {
        for (Coordinate coordinate : object.getOccupies()) {
            this.field[coordinate.getX()][coordinate.getY()] = 1;
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
