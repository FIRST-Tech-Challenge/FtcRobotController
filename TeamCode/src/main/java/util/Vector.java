package util;

public class Vector {
    //x and y coords of the tip of vector, theta is angle measured from the right horizontal
    public double x;
    public double y;
    public double theta;

    //Constructor to create vect using coords
    public Vector(double x1, double y1){
        x = x1;
        y = y1;
        theta = Math.atan2(y, x);
    }
    //Constructor to create vect using angle and length
    public Vector(double angle, double len, angle unit) {
        if (unit.equals(Vector.angle.DEGREES)) {
            angle *= Math.PI/180;
        }
        this.x = len * Math.cos(angle);
        this.y = len * Math.sin(angle);
    }
    //Gets a rotated vector of the current vector angle - positive is anticlockwise
    public Vector getRotatedVec(double angle, angle type) {
        double ang = 0;
        if(type.equals(Vector.angle.DEGREES)) {
            ang = theta + Math.toRadians(angle);
        }else{
            ang = theta + angle;
        }
        double radius = Math.sqrt(x * x + y * y);
        return new Vector(Math.cos(ang) * radius, Math.sin(ang) * radius);
    }
    //Rotates the current vector see above
    public void rotate(double angle, angle type){
        Vector rot = getRotatedVec(angle,type);
        this.x = rot.x;
        this.y = rot.y;
        this.theta = rot.theta;
    }
    //Gets x
    public double getX() {
        return this.x;
    }
    //Gets y
    public double getY() {
        return this.y;
    }
    //Gets length
    public double getLen() {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }
    //Gets angle
    public double getAngle(angle type) {
        if (type == angle.RADIANS) {
            return Math.atan2(y,x);
        } else {
            return Math.atan2(y,x) * 180/Math.PI;
        }
    }
    //Sets x and y coords
    public void setXY(double x1, double y1){
        x = x1;
        y = y1;
        theta = Math.atan2(y, x);
    }
    //Creates a string representation
    public String toString() {
        return "x: " + this.getX() + ", y: " + this.getY() + ", angle: " + this.getAngle(Vector.angle.DEGREES) + ", length: " + this.getLen();
    }


    public Vector add(Vector in){
        return new Vector(x+in.x, y+in.y);
    }

    public Vector subtract(Vector in){
        return new Vector(x-in.x, y-in.y);
    }

    //Enum for angle type
    public enum angle {
        DEGREES,
        RADIANS
    }

}
