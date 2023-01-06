package org.firstinspires.ftc.teamcode.util;

public class Vector3 {
//duck
    public double x;
    public double y;
    public double z;

    public Vector3(double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vector3 add(Vector3 vec){
        return new Vector3(x+vec.x,y+ vec.y, z+vec.z);
    }

    public Vector3 subtract(Vector3 vec){
        return new Vector3(x-vec.x,y- vec.y, z-vec.z);
    }

    public static double magnitude(Vector3 vec){
        return Math.sqrt( Math.pow(vec.x,2) + Math.pow(vec.y,2) + Math.pow(vec.z,2) );
    }

    public String toString(){
        return "(" + x + "," + y + "," + z +")";
    }
}
