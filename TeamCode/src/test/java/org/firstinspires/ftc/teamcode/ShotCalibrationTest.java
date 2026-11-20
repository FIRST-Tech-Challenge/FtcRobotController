package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;

public class ShotCalibrationTest {
    private void hits(ShotCalibration calibration) {
        for (int i = 0; i < 3; i++) calibration.record(true);
    }

    @Test public void competitionProfileUsesTheMeasuredTableWithLinearInterpolation() {
        ShotProfile p = ShootingConfiguration.PROFILE;
        double in = ShotCalibration.METERS_PER_INCH;
        assertTrue(p.calibrated);
        assertEquals(ShootingConfiguration.CALIBRATION_GEOMETRY_ID, p.geometryId);
        double[][] measured = {{12, 2770, 13}, {24, 2890, 8}, {36, 2965, 6}, {48, 3150, 6}, {60, 3190, 0}};
        for (double[] r : measured) {
            assertEquals(r[1], p.sample(r[0] * in).rpm, 1e-9);
            assertEquals(r[2], p.sample(r[0] * in).hoodDeg, 1e-9);
        }
        assertEquals(2830, p.sample(18 * in).rpm, 1e-9);   // halfway 12-24 in
        assertEquals(10.5, p.sample(18 * in).hoodDeg, 1e-9);
        assertEquals(2927.5, p.sample(30 * in).rpm, 1e-9); // halfway 24-36 in
        assertEquals(7, p.sample(30 * in).hoodDeg, 1e-9);
        assertEquals(3057.5, p.sample(42 * in).rpm, 1e-9);
        assertEquals(6, p.sample(42 * in).hoodDeg, 1e-9);
        assertEquals(3170, p.sample(54 * in).rpm, 1e-9);
        assertEquals(3, p.sample(54 * in).hoodDeg, 1e-9);
        // Beyond the measured ends the nearest end segment's line continues.
        assertEquals(2710, p.sample(6 * in).rpm, 1e-9);    // 12-24 in line: +120 RPM / -5 deg per 12 in
        assertEquals(15.5, p.sample(6 * in).hoodDeg, 1e-9);
        assertEquals(2650, p.sample(0).rpm, 1e-9);
        assertEquals(18, p.sample(0).hoodDeg, 1e-9);
        assertEquals(3230, p.sample(72 * in).rpm, 1e-9);   // 48-60 in line: +40 RPM / -6 deg per 12 in
        assertEquals(0, p.sample(72 * in).hoodDeg, 1e-9);   // clamped at hood home
        assertEquals(3270, p.sample(84 * in).rpm, 1e-9);
        assertEquals(0, p.sample(96 * in).hoodDeg, 1e-9);
        assertEquals(ShooterSpeed.FLYWHEEL_MAX_TARGET_RPM, p.sample(1000 * in).rpm, 1e-9); // RPM ceiling
        assertNull(p.sample(-1));
        assertNull(p.sample(Double.NaN));
    }

    @Test public void notebookProfilesStillDoNotExtrapolate() {
        ShotProfile p = new ShotProfile("g", true,
                new ShotProfile.Row(.3, 5, 3000), new ShotProfile.Row(.6, 0, 3200));
        assertFalse(p.extrapolates);
        assertNull(p.sample(.29));
        assertNull(p.sample(.61));
        ShotProfile steep = new ShotProfile("g", true, true,
                new ShotProfile.Row(.3, 0, 3000), new ShotProfile.Row(.6, 20, 1000));
        assertEquals(30, steep.sample(.9).hoodDeg, 1e-9);  // hood clamped to its travel
        assertEquals(1000, steep.sample(.9).rpm, 1e-9);    // RPM floor never drops below the end row
    }

    @Test public void unlimitedConfirmedShotsAverageAcrossSettingsAndMissesAreExcluded() {
        ShotCalibration c = new ShotCalibration();
        c.configure(24, 3200, 0);
        assertNull(c.currentAverage());
        c.record(true); c.record(false);
        assertEquals(1, c.hits());
        c.configure(24, 3400, 10);
        c.record(true);
        assertEquals(3300, c.currentAverage().rpm, 1e-8);
        assertEquals(5, c.currentAverage().hoodDeg, 1e-8);
        for (int i = 0; i < 98; i++) c.record(true);
        assertEquals(100, c.hits());
        assertEquals(3398, c.currentAverage().rpm, 1e-8);
        assertEquals(9.9, c.currentAverage().hoodDeg, 1e-8);
        c.configure(48, 3500, 20);
        c.record(false);
        assertEquals(0, c.hits()); assertNull(c.currentAverage());
        assertEquals(1, c.size());
        c.record(true);
        c.configure(24, 3000, 1);
        assertEquals(100, c.hits()); assertEquals(2, c.size());
    }
    @Test public void fullDownwardTravelCanBeCalibratedAndInterpolated() {
        ShotCalibration c = new ShotCalibration();
        c.configure(24, 3000, 0); hits(c);
        c.configure(48, 3400, 30); hits(c);
        assertEquals(15, c.profile("hood-range").sample(36 * .0254).hoodDeg, 1e-8);
        assertEquals(30, c.profile("hood-range").sample(48 * .0254).hoodDeg, 1e-8);
        assertThrows(IllegalArgumentException.class, () -> c.configure(48, 3400, 30.5));
        assertThrows(IllegalArgumentException.class, () -> c.configure(48, 3400, -.5));
    }

    @Test public void unsortedDistancesInterpolateBothOutputsWithoutExtrapolation() {
        ShotCalibration c = new ShotCalibration();
        c.configure(48, 3400, 10); hits(c);
        c.configure(24, 3000, 2); hits(c);
        ShotProfile p = c.profile("measured-hive");
        assertTrue(p.calibrated);
        ShotProfile.Row mid = p.sample(36 * ShotCalibration.METERS_PER_INCH);
        assertEquals(3200, mid.rpm, 1e-8); assertEquals(6, mid.hoodDeg, 1e-8);
        assertEquals(3000, p.sample(24 * ShotCalibration.METERS_PER_INCH).rpm, 0);
        assertEquals(10, p.sample(48 * ShotCalibration.METERS_PER_INCH).hoodDeg, 0);
        assertNull(p.sample(23 * ShotCalibration.METERS_PER_INCH));
        assertNull(p.sample(49 * ShotCalibration.METERS_PER_INCH));
        c.configure(24, 3100, 3); hits(c);
        assertEquals(2, c.size());
        assertEquals(3050, c.profile("retest").sample(24 * ShotCalibration.METERS_PER_INCH).rpm, 1e-8);
    }

    @Test public void telemetryRowsAreSortedSnapshotsAndANewRunStartsEmpty() {
        ShotCalibration c = new ShotCalibration();
        c.configure(48, 3400, 10); hits(c);
        c.configure(24, 3000, 2); hits(c);
        ShotProfile.Row[] snapshot = c.rows();
        assertEquals(2, snapshot.length);
        assertEquals(24 * .0254, snapshot[0].distance, 1e-8);
        assertEquals(48 * .0254, snapshot[1].distance, 1e-8);
        snapshot[0] = null;
        assertNotNull(c.rows()[0]);
        c.configure(24, 3400, 4); hits(c);
        assertEquals(3200, c.rows()[0].rpm, 1e-8);
        assertEquals(3300, c.profile("live-table").sample(36 * .0254).rpm, 1e-8);
        assertEquals(0, new ShotCalibration().rows().length);
    }
}
