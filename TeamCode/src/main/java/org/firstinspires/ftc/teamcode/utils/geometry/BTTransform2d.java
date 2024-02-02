package org.firstinspires.ftc.teamcode.utils.geometry;

import com.arcrobotics.ftclib.geometry.Transform2d;

/**
 * Represents a transformation for a Pose2d.
 */
public class BTTransform2d extends Transform2d {
    private final BTTranslation2d m_translation;
    private final BTRotation2d m_rotation;

    /**
     * Constructs the transform that maps the initial pose to the final pose.
     *
     * @param initial The initial pose for the transformation.
     * @param last    The final pose for the transformation.
     */
    public BTTransform2d(BTPose2d initial, BTPose2d last) {
        // We are rotating the difference between the translations
        // using a clockwise rotation matrix. This transforms the global
        // delta into a local delta (relative to the initial pose).
        m_translation = last.getTranslation().minus(initial.getTranslation())
                .rotateBy(initial.getRotation().unaryMinus());

        m_rotation = last.getRotation().minus(initial.getRotation());
    }

    /**
     * Constructs a transform with the given translation and rotation components.
     *
     * @param translation Translational component of the transform.
     * @param rotation    Rotational component of the transform.
     */
    public BTTransform2d(BTTranslation2d translation, BTRotation2d rotation) {
        m_translation = translation;
        m_rotation = rotation;
    }

    /**
     * Constructs the identity transform -- maps an initial pose to itself.
     */
    public BTTransform2d() {
        m_translation = new BTTranslation2d();
        m_rotation = new BTRotation2d();
    }

    public BTTransform2d(double x, double y, BTRotation2d rotation2d) {
        this.m_translation = new BTTranslation2d(x,y);
        this.m_rotation = rotation2d;
    }

    /**
     * Scales the transform by the scalar.
     *
     * @param scalar The scalar.
     * @return The scaled Transform2d.
     */
    public BTTransform2d times(double scalar) {
        return new BTTransform2d(m_translation.times(scalar), m_rotation.times(scalar));
    }

    /**
     * Returns the translation component of the transformation.
     *
     * @return The translational component of the transform.
     */
    public BTTranslation2d getTranslation() {
        return m_translation;
    }

    /**
     * Returns the rotational component of the transformation.
     *
     * @return Reference to the rotational component of the transform.
     */
    public BTRotation2d getRotation() {
        return m_rotation;
    }

    @Override
    public String toString() {
        return String.format("Transform2d(%s, %s)", m_translation, m_rotation);
    }

    /**
     * Checks equality between this Transform2d and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal or not.
     */
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof BTTransform2d) {
            return ((BTTransform2d) obj).m_translation.equals(m_translation)
                    && ((BTTransform2d) obj).m_rotation.equals(m_rotation);
        }
        return false;
    }

    /**
     * Invert the transformation. This is useful for undoing a transformation.
     *
     * @return The inverted transformation.
     */
    public BTTransform2d inverse() {
        // We are rotating the difference between the translations
        // using a clockwise rotation matrix. This transforms the global
        // delta into a local delta (relative to the initial pose).
        return new BTTransform2d(getTranslation().unaryMinus().rotateBy(getRotation().unaryMinus()),
                getRotation().unaryMinus());
    }
    public BTTransform2d minus(BTTransform2d other) {
      return new BTTransform2d(this.m_translation.getX()-other.m_translation.getX(),
              this.m_translation.getY()-other.m_translation.getY(),
              this.m_rotation.minus(other.m_rotation));
    }


}