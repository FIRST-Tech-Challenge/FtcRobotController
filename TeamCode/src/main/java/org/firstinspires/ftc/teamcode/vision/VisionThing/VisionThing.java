package org.firstinspires.ftc.teamcode.vision.VisionThing;

public class VisionThing {
    public double x, y, xsize, ysize;
    public String kind;

    public VisionThing(double x, double y, double xsize, double ysize, String kind) {
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
