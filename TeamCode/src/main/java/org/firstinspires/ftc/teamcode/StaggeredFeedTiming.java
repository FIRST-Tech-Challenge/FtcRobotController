package org.firstinspires.ftc.teamcode;

/**
 * Shared stage-1 policy for timed shots. Stage 1 (both linked transfer motors) runs for the whole feed from
 * its start, in both staggered and continuous modes (changed 2026-11-18; it used to wait for the gate-open
 * midpoint while the former stage 2 launched the top ball).
 * In staggered mode the feed itself still starts after the gate delay. Call {@link #feedStage1} every loop,
 * including idle loops. Between held shots, {@link FiringCoordinator#closedGateStage1()} keeps stage 1
 * running with the gate closed.
 */
public final class StaggeredFeedTiming {
    public boolean feedStage1(double now, FiringCoordinator firing) {
        return firing.feeding();
    }

    /** No per-shot state remains; kept so callers can reset on mode or timing changes. */
    public void reset() { }
}
