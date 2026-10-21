import org.firstinspires.ftc.teamcode.pollen.PollenConfig;
import org.firstinspires.ftc.teamcode.pollen.PollenSequence;
import org.firstinspires.ftc.teamcode.pollen.PollenShootTuning;

/** Run with tools/test-pollen-sequence.sh. No Android SDK or simulated FTC classes. */
public final class PollenSequenceTest {
    private static int checks;
    private static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }
    private static void zero(PollenSequence.Output out) {
        check(out.forward == 0 && out.strafe == 0 && out.turn == 0
                && out.launcherVelocity == 0 && out.feederPower == 0, "Nonzero stopped output");
    }
    private static PollenSequence sequence(PollenSequence.Step[] route) {
        return new PollenSequence(route, 1250, 1200, 4, 0.25, 27);
    }
    private static PollenSequence shot() {
        return sequence(new PollenSequence.Step[]{PollenSequence.Step.continuousShoot(2),
                PollenSequence.Step.drive("PARK", 1, 0.2, 0, 0)});
    }
    public static void main(String[] args) {
        PollenSequence park = sequence(new PollenSequence.Step[]{
                PollenSequence.Step.drive("MOVE", .75, .2, 0, 0),
                PollenSequence.Step.drive("HOLD", .5, 0, 0, 0)});
        zero(park.tick(0, 0, false)); // No action before Start.
        park.start(0);
        check(park.tick(0, 0, false).forward == 0.2, "Park movement");
        zero(park.tick(0.75, 0, false));
        PollenSequence.Output done = park.tick(1.25, 0, false);
        check(done.finished, "Park completion");
        zero(done);
        zero(park.tick(2, 0, false));

        PollenSequence shot = shot(); shot.start(0);
        check(shot.tick(0, 0, false).feederPower == 0, "No feeding before speed");
        check(shot.tick(.125, 1250, false).feederPower == 0, "Require stable speed");
        check(shot.tick(.25, 0, false).feederPower == 0, "Reset stability on dip");
        shot.tick(.5, 1250, false);
        check(shot.tick(.75, 1250, false).feederPower == 1, "Feed after stable speed");
        PollenSequence.Output feeding = shot.tick(1, 1100, false);
        check(feeding.feederPower == 1, "Continuous feeding survives an ordinary speed dip");
        check(feeding.activeFeedSeconds == .25, "Count the previous commanded feed interval");
        check(shot.tick(2.74, 1250, false).feederPower == 1, "Full feed window after spinup");
        zero(shot.tick(2.75, 1250, false));
        check(shot.tick(2.76, 0, false).forward == .2, "Park after continuous feed completes");
        check(shot.tick(3.75, 0, false).finished, "Shot route completes");

        PollenSequence delayed = new PollenSequence(new PollenSequence.Step[]{
                PollenSequence.Step.drive("MOVE", 1, .2, 0, 0)}, 1250, 1200, 4, .25, 27, 2);
        delayed.start(10);
        zero(delayed.tick(10, 0, false));
        zero(delayed.tick(11.99, 0, false));
        check(delayed.tick(12, 0, false).forward == .2, "Delay completes before movement");
        check(delayed.tick(13, 0, false).finished, "Delay preserves move duration");
        delayed.start(20);
        zero(delayed.tick(21, 0, true));
        check(delayed.tick(22, 0, false).finished, "Stop during delay is latched");

        PollenSequence timeout = shot(); timeout.start(0);
        timeout.tick(0, 0, false);
        zero(timeout.tick(4, 0, false));
        check(!timeout.warning().isEmpty(), "Timeout reported");
        check(timeout.tick(4.01, 0, false).forward == 0.2, "Park after launcher timeout");

        // Stop before start, during driving, spinup, feeding, and parking must latch zero.
        for (double stopAt : new double[]{0, 0.5, 1.5, 2.8, 4.5, 6.0}) {
            PollenSequence s = shot(); s.start(0);
            for (double t = 0; t < stopAt; t += 0.02) s.tick(t, 1250, false);
            zero(s.tick(stopAt, 1250, true));
            PollenSequence.Output stopped = s.tick(stopAt + 0.1, 1250, false);
            zero(stopped); check(stopped.finished, "Stop latched");
        }
        PollenSequence stoppedBeforeStart = shot();
        zero(stoppedBeforeStart.tick(0, 0, true));
        check(stoppedBeforeStart.tick(1, 0, false).finished, "Stop before start latched");

        PollenSequence deadline = shot(); deadline.start(0);
        zero(deadline.tick(27, 1250, false));
        check(deadline.tick(28, 1250, false).finished, "Hard deadline latched");
        PollenSequence clock = shot(); clock.start(2);
        check(clock.tick(1, 1250, false).finished, "Bad clock stops");

        PollenSequence diagonal = sequence(new PollenSequence.Step[]{
                PollenSequence.Step.drive("diagonal", 1, 1, 1, 1)});
        diagonal.start(0);
        double[] wheels = PollenSequence.wheelPowers(diagonal.tick(0, 0, false));
        for (double power : wheels) check(Math.abs(power) <= 1, "Motor normalization");
        check(wheels[0] == 1 && wheels[1] < 0, "Mecanum signs");

        boolean rejected = false;
        try { sequence(new PollenSequence.Step[]{PollenSequence.Step.drive("too long", 27, .2, 0, 0)}); }
        catch (IllegalArgumentException expected) { rejected = true; }
        check(rejected, "Reject route beyond deadline");
        rejected = false;
        try { PollenSequence.Step.drive("NaN", 1, Double.NaN, 0, 0); }
        catch (IllegalArgumentException expected) { rejected = true; }
        check(rejected, "Reject NaN route");

        // End-to-end both real route definitions with successful / failed simulated shooter.
        for (boolean shoot : new boolean[]{false, true}) {
            for (boolean failure : new boolean[]{false, true}) {
                PollenSequence s = shoot ? new PollenShootTuning(null).sequence()
                        : PollenSequence.park(PollenConfig.parkRoute(),
                                PollenConfig.TOTAL_TIME_LIMIT, PollenConfig.START_DELAY_SECONDS);
                s.start(0); boolean finished = false; boolean fed = false;
                for (double t = 0; t < PollenConfig.TOTAL_TIME_LIMIT + 1; t += .02) {
                    PollenSequence.Output out = s.tick(t, failure ? 0 : 1250, false);
                    fed |= out.feederPower > 0;
                    if (out.finished) { zero(out); finished = true; break; }
                }
                check(finished, "Full route finishes");
                check(fed == (shoot && !failure), "Expected feed outcome");
            }
        }
        rejected = false;
        try { sequence(new PollenSequence.Step[]{PollenSequence.Step.continuousShoot(23)}); }
        catch (IllegalArgumentException expected) { rejected = true; }
        check(rejected, "Budget includes spinup plus continuous feed");
        rejected = false;
        try { PollenSequence.Step.continuousShoot(0); }
        catch (IllegalArgumentException expected) { rejected = true; }
        check(rejected, "Reject zero-length shot step");
        rejected = false;
        try { PollenSequence.Step.continuousShoot(Double.NaN); }
        catch (IllegalArgumentException expected) { rejected = true; }
        check(rejected, "Reject invalid feed duration");
        rejected = false;
        try { PollenSequence.park(new PollenSequence.Step[]{PollenSequence.Step.continuousShoot(1)}, 27, 0); }
        catch (IllegalArgumentException expected) { rejected = true; }
        check(rejected, "Drive-only sequence rejects shooting steps");
        for (double delay : new double[]{-1, Double.NaN, 25}) {
            rejected = false;
            try { new PollenSequence(new PollenSequence.Step[]{PollenSequence.Step.drive("MOVE", 1, .2, 0, 0)},
                    1250, 1200, 4, .25, 27, delay); }
            catch (IllegalArgumentException expected) { rejected = true; }
            check(rejected, "Reject invalid or over-budget delay");
        }
        System.out.println("PASS: " + checks + " checks (pure Java sequence; not Android/hardware validation)");
    }
}
