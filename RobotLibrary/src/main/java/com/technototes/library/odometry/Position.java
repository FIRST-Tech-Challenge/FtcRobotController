package com.technototes.library.odometry;

/** Holds a position object
 * @author Alex Stedman and Ryan Tiotuico
 */
public class Position {
    private double x;
    private double y;
    private double rotation;

    /**
     * Create position
     */
    public Position(){
        new Position(0, 0, 0);
    }

    /**
     *  Create position
     * @param startingx the starting x value
     * @param startingy the starting y value
     * @param startingRotation starting rotation value
     */
    public Position(double startingx, double startingy, double startingRotation){
        x = startingx;
        y = startingy;
        rotation = startingRotation;
    }

    /**
     *
     * @return the x value
     */
    public double getX() {
        return x;
    }

    /**
     *
     * @return the y value
     */
    public double getY() {
        return y;
    }

    /**
     *
     * @return the rotation value
     */
    public double getRotation() {
        return rotation;
    }

    /** adds new values to current position
     *
     * @param deltaX
     * @param deltaY
     * @param deltaRotation
     * @return this
     */
    public Position update(double deltaX, double deltaY, double deltaRotation){
        x+=deltaX;
        y+=deltaY;
        rotation+=deltaRotation;
        return this;
    }

    /** get the other position relative to this one
     *
     * @param other the other position
     * @return the new position relative
     */
    public Position compare(Position other){
        return new Position(other.getX()-x, other.getY()-y, other.getRotation()-rotation);
    }
}
