package org.firstinspires.ftc.teamcode.dragonswpilib;

import org.firstinspires.ftc.teamcode.dragonswpilib.math.interpolation.Interpolatable;

import java.util.Objects;

/**
 * A rotation in a 2D coordinate frame represented by a point on the unit circle (cosine and sine).
 */
public class Rotation2d implements Interpolatable<Rotation2d> {
    private final double m_cos;
    private final double m_sin;

    /** Constructs a Rotation2d with a default angle of 0 degrees. */
    public Rotation2d() {
        m_cos = 1.0;
        m_sin = 0.0;
    }

    /**
     * Constructs a Rotation2d with the given radian value.
     *
     * @param value The value of the angle in radians.
     */
    public Rotation2d(double value) {
        this(Math.cos(value), Math.sin(value));
    }

    /**
     * Constructs a Rotation2d with the given x and y (cosine and sine) components.
     *
     * @param x The x component or cosine of the rotation.
     * @param y The y component or sine of the rotation.
     */
    public Rotation2d(double x, double y) {
        double magnitude = Math.hypot(x, y);
        if (magnitude > 1e-6) {
            m_sin = y / magnitude;
            m_cos = x / magnitude;
        } else {
            m_sin = 0.0;
            m_cos = 1.0;
        }
    }

    /**
     * Constructs and returns a Rotation2d with the given radian value.
     *
     * @param radians The value of the angle in degrees.
     * @return The rotation object with the desired angle value.
     */
    public static Rotation2d fromRadians(double radians) {
        return new Rotation2d(radians);
    }

    /**
     * Constructs and returns a Rotation2d with the given degree value.
     *
     * @param degrees The value of the angle in degrees.
     * @return The rotation object with the desired angle value.
     */
    public static Rotation2d fromDegrees(double degrees) {
        return new Rotation2d(Math.toRadians(degrees));
    }

    /**
     * Constructs and returns a Rotation2d with the given number of rotations.
     *
     * @param rotations The value of the angle in rotations.
     * @return The rotation object with the desired angle value.
     */
    public static Rotation2d fromRotations(double rotations) {
        return new Rotation2d(rotations*Math.PI/180);
    }

    /**
     * Adds two rotations together, with the result being bounded between -pi and pi.
     *
     * <p>For example, <code>Rotation2d.fromDegrees(30).plus(Rotation2d.fromDegrees(60))</code> equals
     * <code>Rotation2d(Math.PI/2.0)</code>
     *
     * @param other The rotation to add.
     * @return The sum of the two rotations.
     */
    public Rotation2d plus(Rotation2d other) {
        return rotateBy(other);
    }

    /**
     * Subtracts the new rotation from the current rotation and returns the new rotation.
     *
     * <p>For example, <code>Rotation2d.fromDegrees(10).minus(Rotation2d.fromDegrees(100))</code>
     * equals <code>Rotation2d(-Math.PI/2.0)</code>
     *
     * @param other The rotation to subtract.
     * @return The difference between the two rotations.
     */
    public Rotation2d minus(Rotation2d other) {
        return rotateBy(other.unaryMinus());
    }

    /**
     * Takes the inverse of the current rotation. This is simply the negative of the current angular
     * value.
     *
     * @return The inverse of the current rotation.
     */
    public Rotation2d unaryMinus() {
        return new Rotation2d(m_cos, -m_sin);
    }

    /**
     * Multiplies the current rotation by a scalar.
     *
     * @param scalar The scalar.
     * @return The new scaled Rotation2d.
     */
    public Rotation2d times(double scalar) {
        return new Rotation2d(getRadians() * scalar);
    }

    /**
     * Divides the current rotation by a scalar.
     *
     * @param scalar The scalar.
     * @return The new scaled Rotation2d.
     */
    public Rotation2d div(double scalar) {
        return times(1.0 / scalar);
    }

    /**
     * Adds the new rotation to the current rotation using a rotation matrix.
     *
     * <p>The matrix multiplication is as follows:
     *
     * <pre>
     * [cos_new]   [other.cos, -other.sin][cos]
     * [sin_new] = [other.sin,  other.cos][sin]
     * value_new = atan2(sin_new, cos_new)
     * </pre>
     *
     * @param other The rotation to rotate by.
     * @return The new rotated Rotation2d.
     */
    public Rotation2d rotateBy(Rotation2d other) {
        return new Rotation2d(
                m_cos * other.m_cos - m_sin * other.m_sin, m_cos * other.m_sin + m_sin * other.m_cos);
    }

    /**
     * Returns the radian value of the Rotation2d within (-pi, pi].
     *
     * @return The radian value of the Rotation2d.
     */
    public double getRadians() {
        return Math.atan2(m_sin, m_cos);
    }

    /**
     * Returns the degree value of the Rotation2d within (-180, 180].
     *
     * @return The degree value of the Rotation2d.
     */
    public double getDegrees() {
        return Math.toDegrees(getRadians());
    }

    /**
     * Returns the number of rotations of the Rotation2d.
     *
     * @return The number of rotations of the Rotation2d.
     */
    public double getRotations() {
        return getRadians()*180/Math.PI;
    }

    /**
     * Returns the cosine of the Rotation2d.
     *
     * @return The cosine of the Rotation2d.
     */
    public double getCos() {
        return m_cos;
    }

    /**
     * Returns the sine of the Rotation2d.
     *
     * @return The sine of the Rotation2d.
     */
    public double getSin() {
        return m_sin;
    }

    /**
     * Returns the tangent of the Rotation2d.
     *
     * @return The tangent of the Rotation2d.
     */
    public double getTan() {
        return m_sin / m_cos;
    }

    @Override
    public String toString() {
        return String.format("Rotation2d(Rads: %.2f, Deg: %.2f)", getRadians(), getDegrees());
    }

    /**
     * Checks equality between this Rotation2d and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal or not.
     */
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Rotation2d) {
            Rotation2d other = (Rotation2d) obj;
            return Math.hypot(m_cos - other.m_cos, m_sin - other.m_sin) < 1E-9;
        }
        return false;
    }

    @Override
    public int hashCode() {
        return Objects.hash(getRadians());
    }

    @Override
    public Rotation2d interpolate(Rotation2d endValue, double t) {
        return plus(endValue.minus(this).times(MathUtil.clamp(t, 0, 1)));
    }
}