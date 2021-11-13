package org.firstinspires.ftc.teamcode.Enhancement.Subsystems.Drive;

import androidx.annotation.NonNull;

/**
 * The Odometry class has two TrcPoses, one for position and another for velocity
 *
 * <p>The TrcPose2D's are edited with get and set methods</p>
 *
 * @see TrcPose2D
 */
public class Odometry {
    private TrcPose2D position;
    private TrcPose2D velocity;

    /**
     * Constructor: Create an instance of the object.
     *
     * <p>Defaults to a default TrcPose2D, position 0 and velocity 0.</p>
     *
     * @see TrcPose2D#TrcPose2D()
     */
    Odometry() {
        position = new TrcPose2D();
        velocity = new TrcPose2D();
    }

    /**
     * Constructor: Create an instance of the object.
     *
     * @param position specifies the initial position.
     * @param velocity specifies the initial velocity.
     */
    Odometry(TrcPose2D position, TrcPose2D velocity) {
        this.position = position;
        this.velocity = velocity;
    }

    /**
     * This method returns the string representation of the object, it is dependent on the {@link TrcPose2D#toString()} method.
     *
     * @return string representation of the object.
     * @see TrcPose2D#toString()
     */
    @Override
    public String toString() {
        return "position = " + position.toString() + ", velocity = " + velocity.toString();
    }

    /**
     * This method creates and returns a copy of this odometry.
     *
     * @return a copy of this odometry.
     */
    @NonNull
    public Odometry clone() {
        return new Odometry(position.clone(), velocity.clone());
    }


    /**
     * @return the position TrcPose2D.
     */
    public TrcPose2D getPosition() {
        return this.position;
    }

    /**
     * This method sets the position info of the odometry to the given pose.
     *
     * @param pose specifies the pose to set the position info to.
     */
    public void setPosition(TrcPose2D pose) {
        this.position.setAs(pose);
    }

    /**
     * This method gets the velocity info of the odometry.
     *
     * @return The velocity pose
     */
    public TrcPose2D getVelocity() {
        return this.velocity;
    }


    /**
     * This method sets the velocity info of the odometry to the given pose.
     *
     * @param pose specifies the pose to set the velocity info to.
     */
    public void setVelocity(TrcPose2D pose) {
        this.velocity.setAs(pose);
    }

}
