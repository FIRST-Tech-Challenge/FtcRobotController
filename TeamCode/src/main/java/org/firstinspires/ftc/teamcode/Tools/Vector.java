package org.firstinspires.ftc.teamcode.Tools;


public class Vector {


    private double i;
    private double j;


    // Default constructor (zero vector)
    public Vector() {
        i = 0;
        j = 0;
    }


    // Parameterized constructor
    public Vector(double i, double j) {
        this.i = i;
        this.j = j;
    }


    // Getter for 'i' component
    public double getI() {
        return i;
    }


    // Getter for 'j' component
    public double getJ() {
        return j;
    }


    // Setter for 'i' component
    public void setI(double i) {
        this.i = i;
    }


    // Setter for 'j' component
    public void setJ(double j) {
        this.j = j;
    }


    // Scale the vector by a scalar value
    public void scalarMultiple(double s) {
        this.i *= s;
        this.j *= s;
    }


    // Return a new vector scaled by a scalar
    public Vector scale(double s) {
        return new Vector(this.i * s, this.j * s);
    }


    // Compute the dot product with another vector
    public double dot(Vector u) {
        return i * u.getI() + j * u.getJ();
    }


    // Compute the magnitude of the vector
    public double magnitude() {
        return Math.sqrt(this.dot(this));
    }


    // Normalize the vector (make its magnitude 1)
    public Vector normalize() {
        double mag = this.magnitude();
        if (mag != 0) {
            return new Vector(this.i / mag, this.j / mag);
        }
        return new Vector(0, 0); // Return zero vector if original vector is zero
    }


    // Add another vector to this vector
    public Vector add(Vector u) {
        return new Vector(this.i + u.getI(), this.j + u.getJ());
    }


    // Subtract another vector from this vector
    public Vector subtract(Vector u) {
        return new Vector(this.i - u.getI(), this.j - u.getJ());
    }


    // Compute the angle of the vector (in radians)
    public double angle() {
        return Math.atan2(j, i);
    }


    // Rotate the vector by a given angle (in radians)
    public Vector rotate(double angle) {
        double cosA = Math.cos(angle);
        double sinA = Math.sin(angle);
        double newI = i * cosA - j * sinA;
        double newJ = i * sinA + j * cosA;
        return new Vector(newI, newJ);
    }


    // Override toString for easy debugging
    @Override
    public String toString() {
        return "Vector(" + i + ", " + j + ")";
    }
}
