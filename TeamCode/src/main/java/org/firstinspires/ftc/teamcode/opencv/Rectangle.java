package org.firstinspires.ftc.teamcode.opencv;

public class Rectangle {
    public int x, y, width, height;

    public Rectangle(int x1, int y1, int x2, int y2) {
        x = x1;
        y = y1;
        width = x2 - x1;
        height = y2 - y1;
    }

    public double area() {
        return width * height;
    }

    public boolean contains(int a, int b) {
        return ((x <= a && a < x + width) || (x >= a && a > x + width)) && ((y <= b && b < y + height) || (y >= b && b > y + height));
    }

    public Rectangle fit(Rectangle outer, int i, int j, int noise){
        width -= noise;
        height -= noise;
        Rectangle op1 = new Rectangle(FastDetectSamples.linesArray[i], FastDetectSamples.linesArray[i+1], FastDetectSamples.linesArray[j], FastDetectSamples.linesArray[j+1]);
        Rectangle op2 = new Rectangle(FastDetectSamples.linesArray[j], FastDetectSamples.linesArray[j+1], FastDetectSamples.linesArray[i+2], FastDetectSamples.linesArray[i+3]);
        if (op1.area() > op2.area()){
            if (outer.area() < op1.area()){
                outer = op1;
            }
        }
        else if (outer.area() < op2.area()){
            outer = op2;
        }
        width += noise;
        height += noise;

        return outer;
    }
}