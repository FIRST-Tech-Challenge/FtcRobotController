package org.firstinspires.ftc.teamcode.objects;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

@DisplayName("VelocityController")
class VelocityControllerTest {
    private static final double EPS = 1e-9;

    @Nested
    @DisplayName("feed forward (kF)")
    class FeedForward {
        @Test
        @DisplayName("at the target speed, power is kF x target")
        void atTarget() {
            VelocityController control = new VelocityController(0.000242, 0.003, 0);
            control.SetTargetRpm(2475);
            assertEquals(0.000242 * 2475, control.Calculate(2475, 0.05), EPS);
        }

        @Test
        @DisplayName("target 0 and wheel stopped gives 0 power")
        void zeroTarget() {
            VelocityController control = new VelocityController(0.000242, 0.003, 0.0001);
            control.SetTargetRpm(0);
            assertEquals(0, control.Calculate(0, 0.05), EPS);
        }
    }

    @Nested
    @DisplayName("proportional (kP)")
    class Proportional {
        @Test
        @DisplayName("100 RPM too slow adds kP x 100")
        void tooSlow() {
            VelocityController control = new VelocityController(0.0002, 0.001, 0);
            control.SetTargetRpm(2000);
            assertEquals(0.0002 * 2000 + 0.001 * 100, control.Calculate(1900, 0.05), EPS);
        }

        @Test
        @DisplayName("100 RPM too fast takes away kP x 100")
        void tooFast() {
            VelocityController control = new VelocityController(0.0002, 0.001, 0);
            control.SetTargetRpm(2000);
            assertEquals(0.0002 * 2000 - 0.001 * 100, control.Calculate(2100, 0.05), EPS);
        }
    }

    @Nested
    @DisplayName("integral (kI)")
    class Integral {
        @Test
        @DisplayName("staying too slow makes the power grow")
        void grows() {
            VelocityController control = new VelocityController(0, 0, 0.001);
            control.SetTargetRpm(1000);
            double first = control.Calculate(900, 0.05);
            double second = control.Calculate(900, 0.05);
            assertEquals(0.001 * 100 * 0.05, first, EPS);
            assertEquals(0.001 * 100 * 0.10, second, EPS);
        }

        @Test
        @DisplayName("changing the target starts the integral over")
        void resetOnNewTarget() {
            VelocityController control = new VelocityController(0, 0, 0.001);
            control.SetTargetRpm(1000);
            control.Calculate(900, 1.0);
            control.SetTargetRpm(1050);
            assertEquals(0.001 * 150 * 0.05, control.Calculate(900, 0.05), EPS);
        }

        @Test
        @DisplayName("setting the same target again does not start it over")
        void sameTargetKeepsIntegral() {
            VelocityController control = new VelocityController(0, 0, 0.001);
            control.SetTargetRpm(1000);
            control.Calculate(900, 1.0);
            control.SetTargetRpm(1000);
            assertEquals(0.001 * 100 * 1.05, control.Calculate(900, 0.05), EPS);
        }

        @Test
        @DisplayName("ClearIntegral forgets the error so far")
        void clear() {
            VelocityController control = new VelocityController(0, 0, 0.001);
            control.SetTargetRpm(1000);
            control.Calculate(900, 1.0);
            control.ClearIntegral();
            assertEquals(0.001 * 100 * 0.05, control.Calculate(900, 0.05), EPS);
        }
    }

    @Nested
    @DisplayName("power limits")
    class Limits {
        @Test
        @DisplayName("never negative, even far above the target")
        void neverNegative() {
            VelocityController control = new VelocityController(0.0002, 0.003, 0);
            control.SetTargetRpm(1000);
            assertEquals(0, control.Calculate(5000, 0.05), EPS);
        }

        @Test
        @DisplayName("never above full power, even far below the target")
        void neverAboveOne() {
            VelocityController control = new VelocityController(0.0002, 0.003, 0);
            control.SetTargetRpm(5000);
            assertEquals(1, control.Calculate(0, 0.05), EPS);
        }
    }
}
