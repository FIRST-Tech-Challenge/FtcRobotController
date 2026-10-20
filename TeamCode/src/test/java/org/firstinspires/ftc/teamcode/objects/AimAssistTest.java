package org.firstinspires.ftc.teamcode.objects;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/** The turn toward a tag: kP 0.02 per degree, never past 0.6, done within 1 degree. */
@DisplayName("AimAssist")
class AimAssistTest {
    private static final double EPS = 1e-9;

    private double turn(double bearing) {
        return AimAssist.RotateToward(bearing, 0.02, 0.6, 1.0);
    }

    @Test
    @DisplayName("tag to the left (positive bearing): turn left (positive)")
    void leftTurnsLeft() {
        assertEquals(0.4, turn(20), EPS);
    }

    @Test
    @DisplayName("tag to the right: turn right")
    void rightTurnsRight() {
        assertEquals(-0.4, turn(-20), EPS);
    }

    @Test
    @DisplayName("never turns harder than the limit, either way")
    void limited() {
        assertEquals(0.6, turn(90), EPS);
        assertEquals(-0.6, turn(-90), EPS);
    }

    @Test
    @DisplayName("close enough to center: stop turning, so the robot does not wobble")
    void doneNearCenter() {
        assertEquals(0, turn(0.5), EPS);
        assertEquals(0, turn(-1.0), EPS);
        assertEquals(0, turn(0), EPS);
    }

    @Test
    @DisplayName("a broken number (NaN) turns nothing")
    void nanTurnsNothing() {
        assertEquals(0, turn(Double.NaN), EPS);
    }

    @Test
    @DisplayName("a negative kP flips the direction, for a camera that reads backwards")
    void negativeKpFlips() {
        assertEquals(-0.4, AimAssist.RotateToward(20, -0.02, 0.6, 1.0), EPS);
    }
}
