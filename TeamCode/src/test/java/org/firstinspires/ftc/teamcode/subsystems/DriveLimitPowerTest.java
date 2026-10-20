package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.firstinspires.ftc.teamcode.bot.BotProperties;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

@DisplayName("Drive.LimitPower (Safe Mode)")
class DriveLimitPowerTest {
    private static final double EPS = 1e-12;

    @Test
    @DisplayName("Safe Mode cap is one quarter")
    void capIsAQuarter() {
        assertEquals(0.25, BotProperties.safeModeMaxPower, EPS);
    }

    @Nested
    @DisplayName("in Safe Mode")
    class Safe {

        @ParameterizedTest(name = "{0} -> {1}")
        @CsvSource({
            "0, 0",
            "1, 0.25",
            "-1, -0.25",
            "0.5, 0.125",
            "-0.4, -0.1",
        })
        @DisplayName("scales the stick range down to a quarter")
        void scales(double in, double out) {
            assertEquals(out, Drive.LimitPower(in, true), EPS);
        }

        @ParameterizedTest(name = "{0} -> {1}")
        @CsvSource({
            "1.41, 0.25",
            "-1.41, -0.25",
            "100, 0.25",
            "Infinity, 0.25",
            "-Infinity, -0.25",
        })
        @DisplayName("clamps anything past full power, like field-centric's 1.41")
        void clamps(double in, double out) {
            assertEquals(out, Drive.LimitPower(in, true), EPS);
        }

        @Test
        @DisplayName("a broken number (NaN) stops the motor")
        void nanStops() {
            assertEquals(0, Drive.LimitPower(Double.NaN, true), EPS);
        }
    }

    @Nested
    @DisplayName("at full speed")
    class Full {

        @ParameterizedTest(name = "{0}")
        @CsvSource({"0", "1", "-1", "0.37", "1.41"})
        @DisplayName("passes power through unchanged")
        void passThrough(double in) {
            assertEquals(in, Drive.LimitPower(in, false), EPS);
        }

        @Test
        @DisplayName("a broken number (NaN) still stops the motor")
        void nanStops() {
            assertEquals(0, Drive.LimitPower(Double.NaN, false), EPS);
        }
    }
}
