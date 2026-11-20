package org.firstinspires.ftc.teamcode;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
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
    // Foresight tuner output measured on V1 (web tuner, second run 2026-11-19), at full wheel power. Run Test: Pedro
    // Drive's hold / line / curve / turn checks before setting TUNING_VERIFIED. Keep autos/robot.json (both copies)
    // in sync.
    public static final double MAX_FORWARD_VELOCITY_IN_PER_SEC = 75.17999283790405;
    public static final double MAX_STRAFE_VELOCITY_IN_PER_SEC = 58.25492114789578;
    public static final double FORWARD_DECELERATION_IN_PER_SEC_SQUARED = 75.93165497310508;
    public static final double STRAFE_DECELERATION_IN_PER_SEC_SQUARED = 112.0086771146408;
    public static final double FORWARD_LINEAR_BRAKE_SECONDS = 0.11286007376738562;
    public static final double STRAFE_LINEAR_BRAKE_SECONDS = 0.052467068492092955;
    public static final double FORWARD_QUADRATIC_BRAKE_SEC_SQUARED_PER_IN = 0.0014572595302026097;
    public static final double STRAFE_QUADRATIC_BRAKE_SEC_SQUARED_PER_IN = 0.0022294994835423856;
    public static final double HEADING_LINEAR_BRAKE_SECONDS = 0.058605172033453154;
    public static final double HEADING_QUADRATIC_BRAKE_SEC_SQUARED_PER_RAD = 0.005840706753898623;
    // Translational gains: primary at or above TRANSLATIONAL_GAIN_SWITCH_IN of error, secondary below it.
    public static final double TRANSLATIONAL_GAIN_SWITCH_IN = 2.5;
    public static final double FORWARD_TRANSLATIONAL_SECONDARY_POWER_PER_IN = 0.14431535947450513;
    public static final double STRAFE_TRANSLATIONAL_SECONDARY_POWER_PER_IN = 0.2164966560007166;
    // The second run reported only the near-path (secondary) gains. Every earlier Foresight tuner run (V1 and
    // master, both axes) gave primary = 2.7065556711758574 x secondary, so the primaries keep that ratio.
    public static final double TRANSLATIONAL_PRIMARY_TO_SECONDARY_RATIO = 2.7065556711758574;
    public static final double FORWARD_TRANSLATIONAL_PRIMARY_POWER_PER_IN = FORWARD_TRANSLATIONAL_SECONDARY_POWER_PER_IN * TRANSLATIONAL_PRIMARY_TO_SECONDARY_RATIO;
    public static final double STRAFE_TRANSLATIONAL_PRIMARY_POWER_PER_IN = STRAFE_TRANSLATIONAL_SECONDARY_POWER_PER_IN * TRANSLATIONAL_PRIMARY_TO_SECONDARY_RATIO;
    public static final double COAST_FEEDFORWARD_POWER_PER_IN_PER_SEC = 0.01281054611286171;
    public static final double BRAKE_FEEDFORWARD_POWER_PER_IN_PER_SEC = 0.010888964195932454;
    public static final double HEADING_POWER_PER_RAD = 6.081142866638771;
    // Path acceleration / deceleration caps (in/s^2). NONE = no limit, Pedro's default; the path planner's
    // Pedro 3 output can also set them per path. Measure before lowering: a cap below the natural
    // deceleration lengthens every stop.
    public static final double MAX_ACCELERATION_IN_PER_SEC_SQUARED = ForesightConfig.Constraint.NONE;
    public static final double MAX_DECELERATION_IN_PER_SEC_SQUARED = ForesightConfig.Constraint.NONE;
    // No speed or wheel-power cap (2026-11-19). The tuner measured braking at full power, and Foresight plans every
    // stop from that braking: a wheel cap below 1 also caps reverse (braking) power, so the robot could not brake as
    // planned, overshot path ends and drove back.
    public static final double DEFAULT_PATH_SPEED_FRACTION = 1;
    public static final double MAX_WHEEL_POWER = 1;
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
            c.headingBrakeCoefficients.set(Vector2D.cartesian(HEADING_LINEAR_BRAKE_SECONDS,
                    HEADING_QUADRATIC_BRAKE_SEC_SQUARED_PER_RAD));
            c.forwardTranslational.set(Controller.piecewise(
                    Controller.proportional(FORWARD_TRANSLATIONAL_SECONDARY_POWER_PER_IN))
                    .put(TRANSLATIONAL_GAIN_SWITCH_IN, Controller.proportional(FORWARD_TRANSLATIONAL_PRIMARY_POWER_PER_IN)));
            c.strafeTranslational.set(Controller.piecewise(
                    Controller.proportional(STRAFE_TRANSLATIONAL_SECONDARY_POWER_PER_IN))
                    .put(TRANSLATIONAL_GAIN_SWITCH_IN, Controller.proportional(STRAFE_TRANSLATIONAL_PRIMARY_POWER_PER_IN)));
            c.coast.set(Controller.proportionalFeedforward(COAST_FEEDFORWARD_POWER_PER_IN_PER_SEC));
            c.brake.set(Controller.proportionalFeedforward(BRAKE_FEEDFORWARD_POWER_PER_IN_PER_SEC));
            c.headingFeedback.set(Controller.proportional(HEADING_POWER_PER_RAD));
            c.maxAccelerationConstraint.set(MAX_ACCELERATION_IN_PER_SEC_SQUARED);
            c.maxDecelerationConstraint.set(MAX_DECELERATION_IN_PER_SEC_SQUARED);
            c.headingConstraint.set(END_HEADING_TOLERANCE_RAD);
            c.translationalConstraint.set(END_TRANSLATIONAL_TOLERANCE_IN);
            c.velocityConstraint.set(END_VELOCITY_TOLERANCE_IN_PER_SEC);
        });
    }
}
