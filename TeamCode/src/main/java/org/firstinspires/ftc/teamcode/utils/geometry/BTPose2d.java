package org.firstinspires.ftc.teamcode.utils.geometry;

import com.arcrobotics.ftclib.geometry.Pose2d;

public class BTPose2d extends Pose2d {
    private final BTTranslation2d m_translation;
    private final BTRotation2d m_rotation;

    /**
     * Constructs a pose at the origin facing toward the positive X axis.
     * (Translation2d{0, 0} and Rotation{0})
     */
    public BTPose2d() {
        m_rotation = new BTRotation2d();
        m_translation = new BTTranslation2d();
    }
    public BTPose2d(Pose2d pose2d){
        m_rotation=BTRotation2d.fromDegrees(pose2d.getRotation().getDegrees());
        m_translation=new BTTranslation2d(pose2d.getX(),pose2d.getY());
    }


    /**
     * Constructs a pose with the specified translation and rotation.
     *
     * @param translation The translational component of the pose.
     * @param rotation    The rotational component of the pose.
     */
    public BTPose2d(BTTranslation2d translation, BTRotation2d rotation) {
        m_translation = translation;
        m_rotation = rotation;
    }

    /**
     * Convenience constructors that takes in x and y values directly instead of
     * having to construct a Translation2d.
     *
     * @param x        The x component of the translational component of the pose.
     * @param y        The y component of the translational component of the pose.
     * @param rotation The rotational component of the pose.
     */
    @SuppressWarnings("ParameterName")
    public BTPose2d(double x, double y, BTRotation2d rotation) {
        m_translation = new BTTranslation2d(x, y);
        m_rotation = rotation;
    }

    /**
     * Transforms the pose by the given transformation and returns the new
     * transformed pose.
     *
     * <p>The matrix multiplication is as follows
     * [x_new]    [cos, -sin, 0][transform.x]
     * [y_new] += [sin,  cos, 0][transform.y]
     * [t_new]    [0,    0,   1][transform.t]
     *
     * @param other The transform to transform the pose by.
     * @return The transformed pose.
     */
    public BTPose2d plus(BTTransform2d other) {
        return transformBy(other);
    }

    /**
     * Returns the Transform2d that maps the one pose to another.
     *
     * @param other The initial pose of the transformation.
     * @return The transform that maps the other pose to the current pose.
     */
    public BTTransform2d minus(BTPose2d other) {
        final BTPose2d pose = this.relativeTo(other);
        return new BTTransform2d(pose.getTranslation(), pose.getRotation());
    }

    /**
     * Returns the translation component of the transformation.
     *
     * @return The translational component of the pose.
     */
    public BTTranslation2d getTranslation() {
        return m_translation;
    }

    /**
     * Returns the rotational component of the transformation.
     *
     * @return The rotational component of the pose.
     */
    public BTRotation2d getRotation() {
        return m_rotation;
    }

    /**
     * Transforms the pose by the given transformation and returns the new pose.
     * See + operator for the matrix multiplication performed.
     *
     * @param other The transform to transform the pose by.
     * @return The transformed pose.
     */
    public BTPose2d transformBy(BTTransform2d other) {
        return new BTPose2d(m_translation.plus(other.getTranslation().rotateBy(m_rotation)),
                m_rotation.plus(other.getRotation()));
    }

    /**
     * Returns the other pose relative to the current pose.
     *
     * <p>This function can often be used for trajectory tracking or pose
     * stabilization algorithms to get the error between the reference and the
     * current pose.
     *
     * @param other The pose that is the origin of the new coordinate frame that
     *              the current pose will be converted into.
     * @return The current pose relative to the new origin pose.
     */
    public BTPose2d relativeTo(BTPose2d other) {
        BTTransform2d transform = new BTTransform2d(other, this);
        return new BTPose2d(transform.getTranslation(), transform.getRotation());
    }

    /**
     * @return the x value from the {@link BTTranslation2d}
     */
    public double getX() {
        return m_translation.getX();
    }

    /**
     * @return the y value from the {@link BTTranslation2d}
     */
    public double getY() {
        return m_translation.getY();
    }

    /**
     * Obtain a new Pose2d from a (constant curvature) velocity.
     *
     * <p>See <a href="https://file.tavsys.net/control/state-space-guide.pdf">
     * Controls Engineering in the FIRST Robotics Competition</a>
     * section on nonlinear pose estimation for derivation.
     *
     * <p>The twist is a change in pose in the robot's coordinate frame since the
     * previous pose update. When the user runs exp() on the previous known
     * field-relative pose with the argument being the twist, the user will
     * receive the new field-relative pose.
     *
     * <p>"Exp" represents the pose exponential, which is solving a differential
     * equation moving the pose forward in time.
     *
     * @param twist The change in pose in the robot's coordinate frame since the
     *              previous pose update. For example, if a non-holonomic robot moves forward
     *              0.01 meters and changes angle by 0.5 degrees since the previous pose update,
     *              the twist would be Twist2d{0.01, 0.0, toRadians(0.5)}
     * @return The new pose of the robot.
     */
    @SuppressWarnings("LocalVariableName")
    public BTPose2d exp(BTTwist2d twist) {
        double dx = twist.dx;
        double dy = twist.dy;
        double dtheta = twist.dtheta;

        double sinTheta = Math.sin(dtheta);
        double cosTheta = Math.cos(dtheta);

        double s;
        double c;
        if (Math.abs(dtheta) < 1E-9) {
            s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
            c = 0.5 * dtheta;
        } else {
            s = sinTheta / dtheta;
            c = (1 - cosTheta) / dtheta;
        }
        BTTransform2d transform = new BTTransform2d(new BTTranslation2d(dx * s - dy * c, dx * c + dy * s),
                new BTRotation2d(cosTheta, sinTheta));

        return this.plus(transform);
    }

    /**
     * Returns a Twist2d that maps this pose to the end pose. If c is the output
     * of a.Log(b), then a.Exp(c) would yield b.
     *
     * @param end The end pose for the transformation.
     * @return The twist that maps this to end.
     */
    public BTTwist2d log(BTPose2d end) {
        final BTPose2d transform = end.relativeTo(this);
        final double dtheta = transform.getRotation().getRadians();
        final double halfDtheta = dtheta / 2.0;

        final double cosMinusOne = transform.getRotation().getCos() - 1;

        double halfThetaByTanOfHalfDtheta;
        if (Math.abs(cosMinusOne) < 1E-9) {
            halfThetaByTanOfHalfDtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
            halfThetaByTanOfHalfDtheta = -(halfDtheta * transform.getRotation().getSin()) / cosMinusOne;
        }

        BTTranslation2d translationPart = transform.getTranslation().rotateBy(
                new BTRotation2d(halfThetaByTanOfHalfDtheta, -halfDtheta)
        ).times(Math.hypot(halfThetaByTanOfHalfDtheta, halfDtheta));

        return new BTTwist2d(translationPart.getX(), translationPart.getY(), dtheta);
    }

    @Override
    public String toString() {
        return String.format("Pose2d(%s, %s)", m_translation, m_rotation);
    }

    /**
     * Checks equality between this Pose2d and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal or not.
     */
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof BTPose2d) {
            return ((BTPose2d) obj).m_translation.equals(m_translation)
                    && ((BTPose2d) obj).m_rotation.equals(m_rotation);
        }
        return false;
    }

    public BTPose2d rotate(double deltaTheta) {
        return new BTPose2d(m_translation, new BTRotation2d(getHeading() + deltaTheta));
    }

    public double getHeading() {
        return m_rotation.getRadians();
    }


}
