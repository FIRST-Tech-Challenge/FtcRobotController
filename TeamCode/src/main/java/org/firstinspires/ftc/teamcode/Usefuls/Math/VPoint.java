package org.firstinspires.ftc.teamcode.Usefuls.Math;

/**
 * Represents a point in a 2D coordinate system.
 */
public class VPoint {

    /**
     * The x-coordinate of the point.
     */
    private double x;

    /**
     * The y-coordinate of the point.
     */
    private double y;

    /**
     * Constructs a new VPoint object with the specified x and y coordinates.
     *
     * @param x the x-coordinate of the point
     * @param y the y-coordinate of the point
     */
    public VPoint(double x, double y){
        this.x = x;
        this.y = y;
    }

    /**
     * Returns the x-coordinate of the point.
     *
     * @return the x-coordinate of the point
     */
    public double getX() {
        return x;
    }

    /**
     * Sets the x-coordinate of the point.
     *
     * @param x the new x-coordinate of the point
     */
    public void setX(double x) {
        this.x = x;
    }

    /**
     * Returns the y-coordinate of the point.
     *
     * @return the y-coordinate of the point
     */
    public double getY() {
        return y;
    }

    /**
     * Sets the y-coordinate of the point.
     *
     * @param y the new y-coordinate of the point
     */
    public void setY(double y) {
        this.y = y;
    }

    /**
     * Creates and returns a copy of this VPoint object.
     *
     * @return a new VPoint object with the same x and y coordinates as this VPoint object
     */
    public VPoint clone(){
        return new VPoint(x,y);
    }

}
