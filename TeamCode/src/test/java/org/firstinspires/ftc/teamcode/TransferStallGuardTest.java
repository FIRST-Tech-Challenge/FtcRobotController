package org.firstinspires.ftc.teamcode;

import org.junit.Test;

import static org.junit.Assert.*;

/** Stage 1 thermal protection: sustained low-speed high-current stalls cut, cool, retry, then lock out. */
public class TransferStallGuardTest {
    private static final double STALL_RPM = 100, STALL_AMPS = 9, FREE_RPM = 5800, FREE_AMPS = .3;
    private static final double DT = .02;

    @Test public void freeRunningAndLoadedRunningKeepFullPower() {
        TransferStallGuard g = new TransferStallGuard();
        for (double t = 0; t < 5; t += DT) assertEquals(1, g.filter(t, 1, FREE_RPM, FREE_AMPS), 0);
        // Hard but moving load: high current at a healthy speed is not a stall.
        for (double t = 5; t < 10; t += DT) assertEquals(1, g.filter(t, 1, 3000, 5), 0);
        assertEquals(TransferStallGuard.State.OK, g.state());
    }

    @Test public void unpluggedEncoderOrUnsampledCurrentNeverTrips() {
        TransferStallGuard g = new TransferStallGuard();
        for (double t = 0; t < 5; t += DT) assertEquals(1, g.filter(t, 1, 0, 2), 0); // 0 RPM, normal current.
        for (double t = 5; t < 10; t += DT) assertEquals(1, g.filter(t, 1, 0, Double.NaN), 0);
        assertEquals(0, g.totalTrips());
    }

    @Test public void zeroEncoderReadingAtOrdinaryLoadingCurrentNeverTrips() {
        // Pushing balls at about 3500 RPM draws 4-5 A; a 0 RPM encoder reading must not turn that into
        // stop-start cycling while loading (seen on the robot with the 4 A threshold, 2026-11-18).
        TransferStallGuard g = new TransferStallGuard();
        for (double t = 0; t < 10; t += DT) assertEquals(1, g.filter(t, 1, 0, 5.0), 0);
        assertEquals(TransferStallGuard.State.OK, g.state());
        assertEquals(0, g.totalTrips());
    }

    @Test public void sustainedStallCutsCoolsRetriesThenLocksOut() {
        TransferStallGuard g = new TransferStallGuard();
        double t = 0;
        for (int trip = 1; trip <= TransferStallGuard.MAX_RETRIES; trip++) {
            double start = t;
            while (t - start < TransferStallGuard.STALL_DETECT_SECONDS - 1e-9) {
                assertEquals("still allowed before the detect time", 1, g.filter(t, 1, STALL_RPM, STALL_AMPS), 0);
                t += DT;
            }
            assertEquals(0, g.filter(t, 1, STALL_RPM, STALL_AMPS), 0);
            assertEquals(trip, g.consecutiveTrips());
            if (trip < TransferStallGuard.MAX_RETRIES) {
                assertEquals(TransferStallGuard.State.COOLING, g.state());
                double coolStart = t;
                t += DT;
                while (t - coolStart < TransferStallGuard.COOLDOWN_SECONDS - DT / 2) {
                    assertEquals("off while cooling", 0, g.filter(t, 1, 0, 0), 0);
                    t += DT;
                }
                t = coolStart + TransferStallGuard.COOLDOWN_SECONDS + DT / 2; // Retry stalls again.
            }
        }
        assertEquals(TransferStallGuard.State.LOCKED_OUT, g.state());
        for (double end = t + 10; t < end; t += DT) assertEquals(0, g.filter(t, 1, 0, 0), 0);
        // Releasing the command (pause, reverse, full) clears the lockout for a fresh attempt.
        assertEquals(0, g.filter(t, 0, 0, 0), 0);
        assertEquals(TransferStallGuard.State.OK, g.state());
        assertEquals(1, g.filter(t + DT, 1, FREE_RPM, FREE_AMPS), 0);
        assertEquals(TransferStallGuard.MAX_RETRIES, g.totalTrips());
    }

    @Test public void cleanRunningAfterRetryForgivesEarlierTrips() {
        TransferStallGuard g = new TransferStallGuard();
        double t = 0;
        while (g.state() == TransferStallGuard.State.OK) { g.filter(t, 1, STALL_RPM, STALL_AMPS); t += DT; }
        assertEquals(1, g.consecutiveTrips());
        t += TransferStallGuard.COOLDOWN_SECONDS;
        for (double end = t + TransferStallGuard.CLEAR_SECONDS + 3 * DT; t < end; t += DT)
            assertEquals(1, g.filter(t, 1, FREE_RPM, FREE_AMPS), 0);
        assertEquals(0, g.consecutiveTrips());
    }

    @Test public void briefStallsShorterThanDetectTimeNeverTrip() {
        TransferStallGuard g = new TransferStallGuard();
        double t = 0;
        for (int i = 0; i < 20; i++) {
            for (double end = t + .5; t < end; t += DT) assertEquals(1, g.filter(t, 1, STALL_RPM, STALL_AMPS), 0);
            for (double end = t + .1; t < end; t += DT) assertEquals(1, g.filter(t, 1, FREE_RPM, FREE_AMPS), 0);
        }
        assertEquals(0, g.totalTrips());
    }
}
