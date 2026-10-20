package org.firstinspires.ftc.teamcode.objects;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import net.jqwik.api.ForAll;
import net.jqwik.api.Label;
import net.jqwik.api.Property;
import net.jqwik.api.constraints.DoubleRange;

/**
 * Property tests for {@link MecanumMath}: rules that must hold for every stick position
 * and heading, not just the hand-picked ones in MecanumMathTest.
 *
 * <p>jqwik only: no JUnit {@code @DisplayName} here, or jqwik silently skips these.
 */
@Label("MecanumMath properties")
class MecanumMathProperties {
    private static final double EPS = 1e-9;

    @Property
    @Label("robot-centric powers stay within [-1, 1] for any stick input")
    void robotCentricInRange(
            @ForAll @DoubleRange(min = -1, max = 1) double drive,
            @ForAll @DoubleRange(min = -1.1, max = 1.1) double strafe,
            @ForAll @DoubleRange(min = -1, max = 1) double rotate) {
        MecanumMath.WheelPowers p = MecanumMath.RobotCentric(drive, strafe, rotate);
        for (double w : new double[] {p.frontLeft, p.frontRight, p.backLeft, p.backRight}) {
            assertTrue(Math.abs(w) <= 1 + EPS, "power " + w);
        }
    }

    @Property
    @Label("field-centric powers never exceed sqrt(2) (documents today's worst case)")
    void fieldCentricBounded(
            @ForAll @DoubleRange(min = -1, max = 1) double drive,
            @ForAll @DoubleRange(min = -1.1, max = 1.1) double strafe,
            @ForAll @DoubleRange(min = -1, max = 1) double rotate,
            @ForAll @DoubleRange(min = -360, max = 360) double yaw) {
        MecanumMath.WheelPowers p = MecanumMath.FieldCentric(drive, strafe, rotate, yaw);
        for (double w : new double[] {p.frontLeft, p.frontRight, p.backLeft, p.backRight}) {
            assertTrue(Math.abs(w) <= Math.sqrt(2) + EPS, "power " + w + " at yaw " + yaw);
        }
    }

    @Property
    @Label("field-centric powers are always finite numbers")
    void fieldCentricFinite(
            @ForAll @DoubleRange(min = -1, max = 1) double drive,
            @ForAll @DoubleRange(min = -1.1, max = 1.1) double strafe,
            @ForAll @DoubleRange(min = -1, max = 1) double rotate,
            @ForAll double yaw) {
        MecanumMath.WheelPowers p = MecanumMath.FieldCentric(drive, strafe, rotate, yaw);
        for (double w : new double[] {p.frontLeft, p.frontRight, p.backLeft, p.backRight}) {
            assertTrue(Double.isFinite(w) || !Double.isFinite(yaw), "power " + w);
        }
    }

    @Property
    @Label("flipping every input flips every wheel (the drive is symmetric)")
    void oddSymmetry(
            @ForAll @DoubleRange(min = -1, max = 1) double drive,
            @ForAll @DoubleRange(min = -1.1, max = 1.1) double strafe,
            @ForAll @DoubleRange(min = -1, max = 1) double rotate,
            @ForAll @DoubleRange(min = -180, max = 180) double yaw) {
        MecanumMath.WheelPowers a = MecanumMath.FieldCentric(drive, strafe, rotate, yaw);
        MecanumMath.WheelPowers b = MecanumMath.FieldCentric(-drive, -strafe, -rotate, yaw);
        assertEquals(-a.frontLeft, b.frontLeft, EPS);
        assertEquals(-a.frontRight, b.frontRight, EPS);
        assertEquals(-a.backLeft, b.backLeft, EPS);
        assertEquals(-a.backRight, b.backRight, EPS);
    }
}
