package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;

public class MecanumDrivetrainTest {
    @Test public void reportedTransmissionHas364MotorRevolutionsPer30WheelRevolutions() {
        assertEquals(364.0 / 30, MecanumDrivetrain.MOTOR_REVS_PER_WHEEL_REV, 1e-12);
        assertEquals(339.7333333333333, MecanumDrivetrain.WHEEL_TICKS_PER_REV, 1e-9);
        assertEquals(28, MecanumDrivetrain.BARE_MOTOR_TICKS_PER_REV, 0);
        assertEquals(5800, MecanumDrivetrain.BARE_MOTOR_MAX_RPM, 0);
        assertEquals(478.02197802197804, MecanumDrivetrain.WHEEL_MAX_RPM, 1e-9);
    }

    @Test public void motorEncoderVelocityConvertsToWheelRpmInBothDirections() {
        // 364 motor RPM is 30 wheel RPM; encoder velocity remains 28 ticks/motor revolution.
        assertEquals(30, MecanumDrivetrain.motorTicksPerSecondToWheelRpm(364.0 * 28 / 60), 1e-9);
        assertEquals(-30, MecanumDrivetrain.motorTicksPerSecondToWheelRpm(-364.0 * 28 / 60), 1e-9);
        assertEquals(0, MecanumDrivetrain.motorTicksPerSecondToWheelRpm(0), 0);
        assertEquals(478.02197802197804,
                MecanumDrivetrain.motorTicksPerSecondToWheelRpm(5800.0 * 28 / 60), 1e-9);
    }

    @Test public void teleOpRateLimitRampsFullReversalWhilePreservingDirection() {
        // Full -1..+1 reversal (2.0 power) takes 0.2 s: a tiny ramp, but no instant torque reversal.
        assertEquals(.2, 2.0 / MecanumDrivetrain.TELEOP_MAX_WHEEL_POWER_CHANGE_PER_SEC, 1e-12);
        double step = .125; // exact in binary so the loop count is deterministic
        double[] reverse = {-1, -1, -1, -1};
        double[] next = MecanumDrivetrain.rateLimitWheelPowers(new double[] {1, 1, 1, 1}, reverse, step);
        assertArrayEquals(new double[] {.875, .875, .875, .875}, next, 0);
        int loops = 1;
        while (next[0] != -1 && loops < 100) { next = MecanumDrivetrain.rateLimitWheelPowers(next, reverse, step); loops++; }
        assertEquals(16, loops);
        assertArrayEquals(reverse, next, 0);
        // Mixed change scales all wheels together; small changes pass straight through.
        double[] mixed = MecanumDrivetrain.rateLimitWheelPowers(new double[4], new double[] {1, -.5, .5, 0}, .1);
        assertArrayEquals(new double[] {.1, -.05, .05, 0}, mixed, 1e-12);
        assertArrayEquals(new double[] {.05, .05, .05, .05},
                MecanumDrivetrain.rateLimitWheelPowers(new double[4], new double[] {.05, .05, .05, .05}, .1), 0);
    }

    @Test public void reported104mmWheelDefinesNominalEncoderDistanceAndNoLoadSpeed() {
        assertEquals(104, MecanumDrivetrain.WHEEL_DIAMETER_MM, 0);
        assertEquals(326.7256359733385,
                MecanumDrivetrain.WHEEL_TICKS_PER_REV / MecanumDrivetrain.MOTOR_TICKS_PER_MM, 1e-9);
        assertEquals(1.039812294867052, MecanumDrivetrain.MOTOR_TICKS_PER_MM, 1e-12);
        assertEquals(2603.0339129744, MecanumDrivetrain.WHEEL_NO_LOAD_SPEED_MM_PER_SEC, 1e-9);
    }
}
