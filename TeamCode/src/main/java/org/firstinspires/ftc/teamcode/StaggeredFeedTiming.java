package org.firstinspires.ftc.teamcode;

/**
 * Shared stage-1 policy for timed shots. Stage 1 starts together with stage 2 at the start of every feed,
 * in both staggered and continuous modes (changed 2026-11-18; it used to wait for the gate-open midpoint).
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
