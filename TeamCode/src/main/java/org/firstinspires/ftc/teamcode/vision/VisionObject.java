package org.firstinspires.ftc.teamcode.vision;

public class VisionObject {
    public double x, y, xsize, ysize;
    public String kind;

    public VisionObject(double x, double y, double xsize, double ysize, String kind) {
        this.x = x;
        this.y = y;
        this.xsize = xsize;
        this.ysize = ysize;
        this.kind = kind;
    }

    public double magSize() {
        return Math.sqrt(Math.pow(xsize,2)+Math.pow(ysize,2));
    }

    public String toString() {
        return kind+" at ("+x+","+y+") with size ("+xsize+","+ysize+") and magSize " + magSize();
    }
}
