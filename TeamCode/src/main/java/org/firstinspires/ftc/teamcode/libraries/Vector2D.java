package org.firstinspires.ftc.teamcode.libraries;

public class Vector2D {
    private double i;  // X component
    private double j;  // Y component
    private double angle;  //in degrees
    private double magnitude; //length of vector

    // Constructor
    public Vector2D(double i, double j) {
        this.i = i;
        this.j = j;
        this.angle = Math.toDegrees(Math.atan2(i,j));
        this.magnitude = Math.sqrt(i*i+j*j);
    }
    public Vector2D(double angle) {
        this.angle = angle;
        this.magnitude = 1;
        this.i = Math.cos(Math.toRadians(angle));
        this.j = Math.sin(Math.toRadians(angle));
    }

    public void setVector(double i, double j) {
        this.i = i;
        this.j = j;
        this.angle = Math.toDegrees(Math.atan2(i,j));
        this.magnitude = Math.sqrt(i*i+j*j);
    }

    public void setAngle(double angle) {
        this.angle = angle;
        this.magnitude = 1;
        this.i = Math.sin(angle*Math.PI/180);
        this.j = Math.cos(angle*Math.PI/180);
    }

    public void adjustAngle(double angle) {
        this.angle = this.angle - angle;
        if (this.angle > 180) {
            this.angle -= 360;
        }

        if (this.angle < -180) {
            this.angle += 360;
        }

        this.i = Math.sin(this.angle * Math.PI / 180.0);
        this.j = Math.cos(this.angle * Math.PI / 180.0);

        if(this.i < .001 && this.i > -.001) {
            this.i = 0;
        }
        if(this.j < .001 && this.j > -.001) {
            this.j = 0;
        }

    }


    public void scaleVector(double scalar) {
        this.i = this.i * scalar;
        this.j = this.j * scalar;
        this.magnitude = Math.abs(this.magnitude * scalar);
        if(scalar == 0) {
            this.angle = 0;
        }

        if(scalar < 0) {
            this.angle += 180;
        }

        if(this.angle > 180) {
            this.angle -= 360;
        }


    }



    // Getters for I and J components
    public double getI() {
        return i;
    }

    public double getJ() {
        return j;
    }

    public void scaleI(double scalar) {
        i = scalar*i;
    }

    public void scaleJ(double scalar) {
        j = scalar*j;
    }

    public double getAngle(){return angle;}

    public double getMagnitude(){return magnitude;}



    public void printVector() {
        System.out.println("i: " + i);
        System.out.println("j: " + j);
        System.out.println("magnitude: " + magnitude);
        System.out.println("Angle: " + angle);

    }


}
