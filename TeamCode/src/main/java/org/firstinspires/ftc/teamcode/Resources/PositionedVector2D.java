package org.firstinspires.ftc.teamcode.Resources;


import com.broombots.sweep.Classes.Pos2D;

/**
 * Represents a two-dimensional vector with a magnitude and a starting position.
 * The position's angle defines the vector's direction. Distances are in inches;
 * angles are in degrees unless otherwise noted.
 */
@SuppressWarnings("unused")
public class PositionedVector2D {
    private double magnitude;
    private final Pos2D POS2D;

    /**
     * Creates a vector with the given magnitude and starting position.
     *
     * @param magnitude vector length in inches
     * @param pos2D starting position and direction; coordinates are in inches
     *              and the angle is in degrees
     */
    public PositionedVector2D(double magnitude, Pos2D pos2D){
        this.magnitude = magnitude;
        this.POS2D = pos2D;
    }

    /** @return vector magnitude in inches */
    public double getMagnitude(){
        return magnitude;
    }

    /** @return vector direction in radians */
    public double getDirectionRadians(){
        return Math.toRadians(POS2D.angle);
    }

    /** @return vector direction in degrees */
    public double getDirectionDegrees(){
        return POS2D.angle;
    }
    /**
     * Rotates the vector by the specified angle.
     *
     * @param degrees rotation in degrees
     * @throws IllegalArgumentException if {@code degrees} is not finite
     */
    public void rotateBy(double degrees){
        if (Double.isNaN(degrees) || Double.isInfinite(degrees)) throw new IllegalArgumentException("Invalid degrees");
        POS2D.angle += degrees;
    }

    /**
     * Sets the rotation of the vector
     * @param degrees from x-axis
     */
    public void setRotationTo(double degrees){
        if (Double.isNaN(degrees) || Double.isInfinite(degrees)) throw new IllegalArgumentException("Invalid degrees");
        POS2D.angle = degrees;
    }

    /**
     * Translates the vector's starting position.
     *
     * @param x horizontal displacement in inches
     * @param y vertical displacement in inches
     * @throws IllegalArgumentException if either displacement is not finite
     */
    public void translateBy(double x, double y){
        if (Double.isNaN(x) || Double.isInfinite(x)) throw new IllegalArgumentException("Invalid Double X");
        if (Double.isNaN(y) || Double.isInfinite(y)) throw new IllegalArgumentException("Invalid Double Y");
        POS2D.x += x;
        POS2D.y += y;
    }
    public void scaleBy(double scalar){
        if (Double.isNaN(scalar) || Double.isInfinite(scalar)) throw new IllegalArgumentException("Invalid Double 'scalar'");
        magnitude *= scalar;
    }

    /** @return the vector's starting position */
    public Pos2D getPos2D(){
        return POS2D;
    }

    /**
     * Calculates the angle from the vector's endpoint to a target position.
     *
     * @param targetPos target position
     * @return angle to the target in degrees
     */
    public double getEndpointAngleToPosition(Pos2D targetPos){
        return Math.toDegrees(
                Math.atan2(
                        targetPos.x-(POS2D.x+magnitude*Math.cos(getDirectionRadians())),
                        targetPos.y - (POS2D.x + magnitude*Math.sin(getDirectionRadians()))
                ));
    }

}
