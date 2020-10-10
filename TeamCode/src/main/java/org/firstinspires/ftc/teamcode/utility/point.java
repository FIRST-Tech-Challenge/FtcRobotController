package org.firstinspires.ftc.teamcode.utility;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class point {
    public double x, y;

    public point(double x, double y){
        this.x = x;
        this.y = y;
    }

    public point rotate(double r){
        return rotate(x, y, r);
    }

    public void translate(double xc, double yc){
        this.x += xc;
        this.y += yc;
    }

    public static point rotate(double x, double y, double r){
        return new point(
                x * cos(r) - y * sin(r),
                y * cos(r) + x * sin(r));
    }

    public double angle(){
        return Math.atan2(y, x);
    }
    public double magnitude(){
        return Math.hypot(y, x);
    }

    public void normalize(){
        double angle = angle();
        x = Math.cos(angle);
        y = Math.sin(angle);
    }

    public void scale(double scale){
        x *= scale;
        y *= scale;
    }
}
