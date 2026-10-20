package org.firstinspires.ftc.teamcode.objects;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/**
 * Smoke tests proving the quality suite's tooling is wired up correctly.
 *
 * <p>This file does not test robot code. It exists so that if the build is ever
 * misconfigured, this fails first with an obvious message, instead of the real tests
 * silently never running. Each check below covers a piece of tooling that later phases
 * depend on:
 *
 * <ul>
 *   <li>Jupiter engine -- runs the ordinary {@code @Test} methods.
 *   <li>Mockito -- needed by the Phase 3 FakeRobot harness.
 *   <li>Mocking SDK types -- proves {@code returnDefaultValues = true} lets us touch
 *       Android/FTC SDK classes from a host-side JVM test with no emulator.
 * </ul>
 *
 * <p>The jqwik engine check lives in {@link JqwikInfrastructureProperties}: jqwik skips
 * properties in a class carrying Jupiter annotations, so the two cannot share a file.
 */
@DisplayName("quality suite infrastructure")
class QualityInfrastructureTest {

    @Test
    @DisplayName("JUnit 5 Jupiter engine runs")
    void jupiterEngineRuns() {
        assertEquals(4, 2 + 2);
    }

    @Test
    @DisplayName("Mockito can create mocks")
    void mockitoWorks() {
        Supplier supplier = mock(Supplier.class);
        when(supplier.value()).thenReturn(42);

        assertEquals(42, supplier.value());
    }

    @Test
    @DisplayName("FTC SDK types can be mocked on the host JVM")
    void canMockSdkTypes() {
        // If returnDefaultValues were false, touching SDK classes here would throw
        // "Method ... not mocked". Phase 3's FakeRobot depends on this working.
        HardwareMap hardwareMap = mock(HardwareMap.class);

        assertNotNull(hardwareMap);
    }

    /** Local stand-in so this smoke test does not depend on any robot class. */
    interface Supplier {
        int value();
    }
}
