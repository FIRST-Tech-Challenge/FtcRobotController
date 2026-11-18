package org.firstinspires.ftc.teamcode;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;

/**
 * Pedro v3 tuning, in inches and radians. Example dynamics must be measured on the
 * 20:80 gear / 30:91 belt drivetrain; unloaded wheel RPM cannot establish loaded dynamics.
 * Pinpoint pod resolution is independent of the drivetrain motor encoders/reduction.
 */
public final class PedroConfiguration {
    private PedroConfiguration() {}
    public static final boolean TUNING_VERIFIED = false;
    public static final double INCHES_PER_METER = 1 / .0254;
    public static final double MM_PER_INCH = 25.4;
    public static final double MAX_FORWARD_VELOCITY_IN_PER_SEC = 40;
    public static final double MAX_STRAFE_VELOCITY_IN_PER_SEC = 35;
    public static final double FORWARD_DECELERATION_IN_PER_SEC_SQUARED = 40;
    public static final double STRAFE_DECELERATION_IN_PER_SEC_SQUARED = 35;
    public static final double FORWARD_LINEAR_BRAKE_SECONDS = .01;
    public static final double STRAFE_LINEAR_BRAKE_SECONDS = .01;
    public static final double FORWARD_QUADRATIC_BRAKE_SEC_SQUARED_PER_IN = .0125;
    public static final double STRAFE_QUADRATIC_BRAKE_SEC_SQUARED_PER_IN = 1.0 / 70;
    // Path acceleration / deceleration caps (in/s^2). NONE = no limit, Pedro's default; the path planner's
    // Pedro 3 output can also set them per path. Measure before lowering: a cap below the natural
    // deceleration lengthens every stop.
    public static final double MAX_ACCELERATION_IN_PER_SEC_SQUARED = ForesightConfig.Constraint.NONE;
    public static final double MAX_DECELERATION_IN_PER_SEC_SQUARED = ForesightConfig.Constraint.NONE;
    public static final double DEFAULT_PATH_SPEED_FRACTION = .35;
    public static final double MAX_WHEEL_POWER = .35;
    public static final double MAX_AUTONOMOUS_SECONDS = 29;
    public static final double END_HEADING_TOLERANCE_RAD = Math.toRadians(3);
    public static final double END_TRANSLATIONAL_TOLERANCE_IN = .5;
    public static final double END_VELOCITY_TOLERANCE_IN_PER_SEC = 1;

    public static ForesightConfig create() {
        return new ForesightConfig(c -> {
            c.maxAchievableForwardVelocity.set(MAX_FORWARD_VELOCITY_IN_PER_SEC);
            c.maxAchievableStrafeVelocity.set(MAX_STRAFE_VELOCITY_IN_PER_SEC);
            c.naturalForwardDeceleration.set(FORWARD_DECELERATION_IN_PER_SEC_SQUARED);
            c.naturalStrafeDeceleration.set(STRAFE_DECELERATION_IN_PER_SEC_SQUARED);
            c.linearBrakeCoefficients.set(Matrix.diag(FORWARD_LINEAR_BRAKE_SECONDS, STRAFE_LINEAR_BRAKE_SECONDS));
            c.quadraticBrakeCoefficients.set(Matrix.diag(FORWARD_QUADRATIC_BRAKE_SEC_SQUARED_PER_IN,
                    STRAFE_QUADRATIC_BRAKE_SEC_SQUARED_PER_IN));
            c.headingBrakeCoefficients.set(Vector2D.zero()); // Measure rotational braking too.
            c.maxAccelerationConstraint.set(MAX_ACCELERATION_IN_PER_SEC_SQUARED);
            c.maxDecelerationConstraint.set(MAX_DECELERATION_IN_PER_SEC_SQUARED);
            c.headingConstraint.set(END_HEADING_TOLERANCE_RAD);
            c.translationalConstraint.set(END_TRANSLATIONAL_TOLERANCE_IN);
            c.velocityConstraint.set(END_VELOCITY_TOLERANCE_IN_PER_SEC);
        });
    }
}
