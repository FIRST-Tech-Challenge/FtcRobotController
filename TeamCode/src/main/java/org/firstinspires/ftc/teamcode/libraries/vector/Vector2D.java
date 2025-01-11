package org.firstinspires.ftc.teamcode.libraries.vector;

public class Vector2D {
    private double i;  // X component
    private double j;  // Y component
    private double radians;  //in degrees
    private double magnitude; //length of vector

    // Constructor
    public Vector2D(double i, double j) {
        this.i = i;
        this.j = j;
        this.magnitude = Math.sqrt(i*i+j*j);
        if(magnitude != 0) {
            this.radians = Math.atan2(j, i);
        } else {
            this.radians = 0;
        }
    }
    public Vector2D(double radians) {
        this.radians = this.radians;
        this.magnitude = 1;
        this.i = Math.cos(radians);
        this.j = Math.sin(radians);

        if(this.i < .001 && this.i > -.001) {
            this.i = 0;
        }

        if(this.j < .001 && this.j > -.001) {
            this.j = 0;
        }

    }

    public void setVector(double i, double j) {
        this.i = i;
        this.j = j;
        this.magnitude = Math.sqrt(i*i+j*j);
        if(magnitude != 0) {
            this.radians = Math.atan2(j, i);
        } else {
            this.radians = 0;
        }
    }

    public void setRadians(double radians) {
        this.radians = radians;
        this.i = Math.cos(radians)*magnitude;
        this.j = Math.sin(radians)*magnitude;

        if(this.i < .001 && this.i > -.001) {
            this.i = 0;
        }
        if(this.j < .001 && this.j > -.001) {
            this.j = 0;
        }

    }
    //gives you the vector relative to a heading e.g. current yaw
    public void setRelative(double radians) {
        this.radians = this.radians - radians;
        if (this.radians > Math.PI) {
            this.radians -= 2*Math.PI;
        }

        if (this.radians < -Math.PI) {
            this.radians += 2*Math.PI;
        }

        this.i = Math.cos(this.radians)*magnitude;
        this.j = Math.sin(this.radians)*magnitude;

        if(this.i < .001 && this.i > -.001) {
            this.i = 0;
        }
        if(this.j < .001 && this.j > -.001) {
            this.j = 0;
        }

    }


    public void rotateVector(double radians) {

        this.radians = this.radians + radians;
        if (this.radians > Math.PI) {
            this.radians -= 2*Math.PI;
        }

        if (this.radians < -Math.PI) {
            this.radians += 2*Math.PI;
        }

        this.i = Math.cos(this.radians)*magnitude;
        this.j = Math.sin(this.radians)*magnitude;

        if(this.i < .001 && this.i > -.001) {
            this.i = 0;
        }
        if(this.j < .001 && this.j > -.001) {
            this.j = 0;
        }

    }


    public void normalizeVector() {
        if (magnitude == 0) {
            return;
        }
        this.i = this.i/this.magnitude;
        this.j = this.j/this.magnitude;
        this.magnitude = 1;
    }

    public void scaleVector(double scalar) {
        this.i = this.i * scalar;
        this.j = this.j * scalar;
        this.magnitude = Math.abs(this.magnitude * scalar);
        if(scalar == 0) {
            this.radians = 0;
        }

        if(scalar < 0) {
            this.radians += 180;
        }

        if(this.radians > 180) {
            this.radians -= 360;
        }


    }



    // Getters for I and J components
    public double getI() {
        return i;
    }

    public double getJ() {
        return j;
    }


    public double getRadians(){return radians;}
    public double getDegrees(){return Math.toDegrees(radians);}

    public double getMagnitude(){return magnitude;}



    public void printVector() {

        System.out.println(i + "i + " + j + "j");

    }

    public void printAllVectorData() {
        System.out.println("i: " + i);
        System.out.println("j: " + j);
        System.out.println("magnitude: " + magnitude);
        System.out.println("Angle: " + radians);

    }

    public static double dotProduct (Vector2D firstVec, Vector2D secondVec) {
        return firstVec.getI() * secondVec.getI() + firstVec.getJ() * secondVec.getJ();
    }


}
