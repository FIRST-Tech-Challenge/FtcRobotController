package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertTrue;

import net.jqwik.api.ForAll;
import net.jqwik.api.Label;
import net.jqwik.api.Property;
import net.jqwik.api.constraints.DoubleRange;
import org.firstinspires.ftc.teamcode.bot.BotProperties;

/** jqwik only: no JUnit {@code @DisplayName} here, or jqwik silently skips these. */
@Label("Drive.LimitPower properties")
class DriveLimitPowerProperties {

    @Property
    @Label("Safe Mode never lets any power past the cap, whatever goes in")
    void neverPastCap(@ForAll double power) {
        double out = Drive.LimitPower(power, true);
        assertTrue(Math.abs(out) <= BotProperties.safeModeMaxPower, power + " -> " + out);
    }

    @Property
    @Label("Safe Mode never flips direction")
    void keepsDirection(@ForAll double power) {
        double out = Drive.LimitPower(power, true);
        assertTrue(out * power >= 0, power + " -> " + out);
    }

    @Property
    @Label("more stick never means less power (the limit is smooth, not jumpy)")
    void monotonic(
            @ForAll @DoubleRange(min = -2, max = 2) double a,
            @ForAll @DoubleRange(min = -2, max = 2) double b) {
        double lo = Math.min(a, b);
        double hi = Math.max(a, b);
        assertTrue(Drive.LimitPower(lo, true) <= Drive.LimitPower(hi, true));
    }
}
