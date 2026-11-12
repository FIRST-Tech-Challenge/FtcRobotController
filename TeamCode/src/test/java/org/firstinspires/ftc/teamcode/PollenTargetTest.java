package org.firstinspires.ftc.teamcode;

import org.junit.Test;

import static org.junit.Assert.*;

public class PollenTargetTest {
    private PollenTarget project(double xPx, double yPx, double forwardOffsetMm, double rightOffsetMm) {
        return PollenTarget.fromFloorContact(1_000_000_000L, xPx, yPx - 10.0, xPx, yPx, 100.0,
                280.0, 280.0, 160.0, 120.0, 180.0, 15.0, forwardOffsetMm, rightOffsetMm);
    }

    @Test public void opticalCenterIntersectsFloorUsingMeasuredHeightAndPitch() {
        PollenTarget target = project(160.0, 120.0, 0.0, 0.0);
        assertNotNull(target);
        assertEquals(180.0 / Math.tan(Math.toRadians(15.0)), target.forwardMm, 1e-6);
        assertEquals(0.0, target.rightMm, 1e-6);
        assertEquals(0.0, target.bearingDeg, 1e-6);
    }

    @Test public void lowerContourMeansCloserFloorContact() {
        assertTrue(project(160.0, 220.0, 0.0, 0.0).forwardMm < project(160.0, 120.0, 0.0, 0.0).forwardMm);
    }

    @Test public void rightAndLeftBearingsMatchClockwiseDriveConvention() {
        assertTrue(project(200.0, 200.0, 0.0, 0.0).bearingDeg > 0.0);
        assertTrue(project(120.0, 200.0, 0.0, 0.0).bearingDeg < 0.0);
    }

    @Test public void cameraOffsetsTranslateIntoRobotCenterCoordinates() {
        PollenTarget baseline = project(160.0, 200.0, 0.0, 0.0);
        PollenTarget shifted = project(160.0, 200.0, 75.0, -20.0);
        assertEquals(baseline.forwardMm + 75.0, shifted.forwardMm, 1e-6);
        assertEquals(-20.0, shifted.rightMm, 1e-6);
    }

    @Test public void aboveHorizonIsRejected() {
        assertNull(project(160.0, 20.0, 0.0, 0.0));
    }

    @Test public void invalidIntrinsicsAndPixelsAreRejected() {
        assertNull(PollenTarget.fromFloorContact(1L, 160, 120, 160, 200, 100,
                0, 280, 160, 120, 180, 15, 0, 0));
        assertNull(project(Double.NaN, 200, 0, 0));
        assertNull(project(160, Double.POSITIVE_INFINITY, 0, 0));
    }
}
