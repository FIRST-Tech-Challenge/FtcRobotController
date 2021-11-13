package org.firstinspires.ftc.teamcode.Util.CoordinateSystem;

public class Coordinate {
    protected int x, y;
    protected double g, h;

    public Coordinate(int x, int y) {
        this.x = x;
        this.y = y;
        this.g = 0;
        this.h = 0;
    }

    public Coordinate(int x, int y, double g, double h) {
        this.x = x;
        this.y = y;
        this.g = g;
        this.h = h;
    }

    public int getX() {
        return x;
    }

    public void setX(int x) {
        this.x = x;
    }

    public int getY() {
        return y;
    }

    public void setY(int y) {
        this.y = y;
    }

    public double getG() {
        return g;
    }

    public void setG(double g) {
        this.g = g;
    }

    public double getH() {
        return h;
    }

    public void setH(double h) {
        this.h = h;
    }

    public boolean equals(Coordinate c) {
        return (this.x == c.x && this.y == c.y);
    }

    public String toString() {
        return "(" + this.x + "," + this.y + ")";
    }

    public Coordinate[] getPossibleMoves() {
        Coordinate[] possible = new Coordinate[8];
        possible[0] = new Coordinate((this.x + 1), (this.y + 1));
        possible[1] = new Coordinate((this.x + 1), (this.y));
        possible[2] = new Coordinate((this.x + 1), (this.y - 1));
        possible[3] = new Coordinate((this.x), (this.y + 1));
        possible[4] = new Coordinate((this.x), (this.y - 1));
        possible[5] = new Coordinate((this.x - 1), (this.y + 1));
        possible[6] = new Coordinate((this.x - 1), (this.y));
        possible[7] = new Coordinate((this.x - 1), (this.y - 1));
        return possible;
    }
}
