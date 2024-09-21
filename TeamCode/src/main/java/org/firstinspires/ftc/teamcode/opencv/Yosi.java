package org.firstinspires.ftc.teamcode.opencv;

import org.opencv.core.Point;

public class Yosi {
    public int x, y, width, height;

    public Yosi(Point p1, Point p2) {
        x = (int) p1.x;
        y = (int) p1.y;
        width = (int) p2.x - x;
        height = (int) p2.y - y;
    }

    public double area() {
        return width * height;
    }

    public boolean contains(Point p) {
        return ((x <= p.x && p.x < x + width) || (x >= p.x && p.x > x + width)) && ((y <= p.y && p.y < y + height) || (y >= p.y && p.y > y + height));
    }
}
