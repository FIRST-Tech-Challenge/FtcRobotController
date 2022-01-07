package org.firstinspires.ftc.teamcode.math;

public class Vector {

    public double x = 0;
    public double y = 0;

    public Vector(double x, double y){
        this.x = x;
        this.y = y;
    }

    public Vector(){
    }

    public double heading(){
        return Math.atan2(y, x);
    }
    public double magnitude(){
        return Math.sqrt(x * x + y * y);
    }
    public void setMagnitude(double mag){
        double Mag = magnitude();
        x = x * mag / Mag;
        y = y * mag / Mag;
    }

    public void mult(double value){
        x*=value;
        y*=value;
    }

    public void set(double x, double y){
        this.x = x;
        this.y = y;
    }

    public static Vector sub(Vector vec1, Vector vec2){
        Vector output = new Vector();
        output.x = vec1.x-vec2.x;
        output.y = vec1.y-vec2.y;
        return output;
    }

    public static Vector normalize(Vector vec){
        double m = vec.magnitude();
        return new Vector(vec.x/m, vec.y/m);
    }

    public static Vector add(Vector vec1, Vector vec2){
        Vector output = new Vector();
        output.x = vec1.x+vec2.x;
        output.y = vec1.y+vec2.y;
        return output;
    }
}