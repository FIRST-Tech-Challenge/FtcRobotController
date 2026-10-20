package org.firstinspires.ftc.teamcode.objects;

import static org.junit.jupiter.api.Assertions.assertEquals;

import net.jqwik.api.ForAll;
import net.jqwik.api.Label;
import net.jqwik.api.Property;
import net.jqwik.api.constraints.IntRange;

/**
 * Smoke test proving the jqwik property-test engine runs.
 *
 * <p>Kept in its own class ON PURPOSE. jqwik is a separate JUnit Platform engine from
 * Jupiter, and it does not understand Jupiter annotations: putting a Jupiter
 * {@code @DisplayName} on a class that also holds {@code @Property} methods makes jqwik
 * SKIP those properties -- quietly, with the build still green. Rule for this suite:
 * property-test classes use jqwik's {@code @Label}, never Jupiter's {@code @DisplayName},
 * and do not mix {@code @Test} and {@code @Property} in one class.
 */
@Label("quality suite infrastructure: jqwik")
class JqwikInfrastructureProperties {

    @Property(tries = 50)
    @Label("jqwik property engine runs")
    void jqwikEngineRuns(@ForAll @IntRange(min = 0, max = 1000) int value) {
        // Trivially true; the point is that the jqwik engine discovered and ran this at all.
        assertEquals(value, Math.abs(value));
    }
}
