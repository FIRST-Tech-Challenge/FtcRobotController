package org.firstinspires.ftc.teamcode.util;

public class Vector2 {

    public double x;
    public double y;

    public Vector2(double x, double y){
        this.x = x;
        this.y = y;
    }

    public Vector2 add(Vector2 vec){
        return new Vector2(x+vec.x,y+ vec.y);
    }

    public Vector2 subtract(Vector2 vec){
        return new Vector2(x-vec.x,y- vec.y);
    }

    public static double magnitude(Vector2 vec){
        return Math.sqrt( Math.pow(vec.x,2) + Math.pow(vec.y,2) );
    }

    public String toString(){
        return "(" + x + "," + y + ")";
    }
}
