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
    // Foresight tuner output from master's pedro/Constants.java, copied into V1 on 2026-11-18 at the user's
    // request so both robots follow paths with the same tuning. It was measured on master's chassis, not V1's
    // 20:80 gear / 30:91 belt drivetrain: re-run Test: Pedro Drive (and the Foresight tuner if paths miss)
    // before setting TUNING_VERIFIED. Keep autos/robot.json (both copies) in sync.
    public static final double MAX_FORWARD_VELOCITY_IN_PER_SEC = 75.791216731181;
    public static final double MAX_STRAFE_VELOCITY_IN_PER_SEC = 60.54276069289256;
    public static final double FORWARD_DECELERATION_IN_PER_SEC_SQUARED = 44.33338627123392;
    public static final double STRAFE_DECELERATION_IN_PER_SEC_SQUARED = 79.85589496541019;
    public static final double FORWARD_LINEAR_BRAKE_SECONDS = 0.10190028205033407;
    public static final double STRAFE_LINEAR_BRAKE_SECONDS = 0.04863765333157554;
    public static final double FORWARD_QUADRATIC_BRAKE_SEC_SQUARED_PER_IN = 0.0015098880451388642;
    public static final double STRAFE_QUADRATIC_BRAKE_SEC_SQUARED_PER_IN = 0.002147552414866776;
    public static final double HEADING_LINEAR_BRAKE_SECONDS = 0.050404207300238334;
    public static final double HEADING_QUADRATIC_BRAKE_SEC_SQUARED_PER_RAD = 0.00662826876091725;
    // Translational gains: primary at or above TRANSLATIONAL_GAIN_SWITCH_IN of error, secondary below it.
    public static final double TRANSLATIONAL_GAIN_SWITCH_IN = 2.5;
    public static final double FORWARD_TRANSLATIONAL_PRIMARY_POWER_PER_IN = 0.3100178262135112;
    public static final double FORWARD_TRANSLATIONAL_SECONDARY_POWER_PER_IN = 0.11454330295701048;
    public static final double STRAFE_TRANSLATIONAL_PRIMARY_POWER_PER_IN = 0.4855766023499335;
    public static final double STRAFE_TRANSLATIONAL_SECONDARY_POWER_PER_IN = 0.17940757972252447;
    public static final double COAST_FEEDFORWARD_POWER_PER_IN_PER_SEC = 0.012715273265700046;
    public static final double BRAKE_FEEDFORWARD_POWER_PER_IN_PER_SEC = 0.010807982275845038;
    public static final double HEADING_POWER_PER_RAD = 5.363935529565826;
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
