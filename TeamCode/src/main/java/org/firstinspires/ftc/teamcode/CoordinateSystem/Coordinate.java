package org.firstinspires.ftc.teamcode.CoordinateSystem;

public class Coordinate {
    int x;
    int y;
    public Coordinate(int x, int y) {
        this.x = x;
        this.y = y;
    }

    public int getX() {
        return x;
    }

    public int getY() {
        return y;
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
