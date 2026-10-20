import org.firstinspires.ftc.teamcode.pollen.PollenStartDelay;
import org.firstinspires.ftc.teamcode.pollen.PollenSequence;

/** Host regression checks for INIT delay controls and their sequencer contract. */
public final class PollenStartDelayTest {
    private static int checks;
    private static void check(boolean result, String message) {
        checks++;
        if (!result) throw new AssertionError(message);
    }
    private static void press(PollenStartDelay delay, boolean up) {
        delay.update(false, false);
        delay.update(up, !up);
    }
    private static void zero(PollenSequence.Output out) {
        check(out.forward == 0 && out.strafe == 0 && out.turn == 0
                && out.launcherVelocity == 0 && out.feederPower == 0, "Wait/stop must request zero");
    }
    public static void main(String[] args) {
        PollenStartDelay delay = new PollenStartDelay(0, 1.25, 27);
        check(delay.delaySeconds() == 0, "Default works without a controller");
        check(delay.maximumDelaySeconds() == 20, "Long waits capped at 20 seconds");
        press(delay, true);
        check(delay.delaySeconds() == .5, "Up increases by half second");
        for (int i = 0; i < 100; i++) delay.update(true, false);
        check(delay.delaySeconds() == .5, "Holding up does not repeat");
        press(delay, true);
        check(delay.delaySeconds() == 1, "Release and re-press increments");
        press(delay, false);
        check(delay.delaySeconds() == .5, "Down decreases");
        delay.update(false, true);
        check(delay.delaySeconds() == .5, "Holding down does not repeat");
        delay.update(false, false);
        delay.update(true, true);
        check(delay.delaySeconds() == .5, "Opposing inputs ignored");
        for (int i = 0; i < 100; i++) press(delay, false);
        check(delay.delaySeconds() == 0, "Delay cannot go negative");
        for (int i = 0; i < 100; i++) press(delay, true);
        check(delay.delaySeconds() == 20, "Delay cannot exceed maximum");
        delay.lock();
        press(delay, false);
        check(delay.locked() && delay.delaySeconds() == 20, "START freezes selected delay");

        PollenSequence.Step[] route = {PollenSequence.Step.drive("FORWARD", 1, .2, 0, 0),
                PollenSequence.Step.drive("HOLD", .25, 0, 0, 0)};
        double duration = PollenSequence.worstCaseDuration(route, 4);
        check(duration == 1.25, "Route budget includes every leg");
        PollenStartDelay selected = new PollenStartDelay(0, duration, 27);
        for (int i = 0; i < 4; i++) press(selected, true);
        selected.lock();
        PollenSequence sequence = new PollenSequence(route, 1250, 1200, 4, .25, 27, selected.delaySeconds());
        sequence.start(0);
        zero(sequence.tick(0, 0, false));
        zero(sequence.tick(1.99, 0, false));
        check(sequence.tick(2, 0, false).forward == .2, "Selected delay precedes movement");
        press(selected, true);
        check(selected.delaySeconds() == 2, "Post-START input cannot alter delay");
        check(sequence.tick(2.99, 0, false).forward == .2, "Delay does not shorten drive duration");
        zero(sequence.tick(3, 0, false));
        check(sequence.tick(3.25, 0, false).finished, "Hold and finish after delayed drive");
        sequence.start(0);
        zero(sequence.tick(1, 0, true));
        check(sequence.tick(3, 0, false).finished, "STOP during delay stays stopped");
        check(new PollenStartDelay(0, duration, 27).delaySeconds() == 0, "Fresh INIT resets to configured default");

        // Every selectable delay must pass the exact sequencer budget, including long routes.
        for (double seconds : new double[]{1, 6, 10.5, 25.75}) {
            PollenSequence.Step[] timed = {PollenSequence.Step.drive("MOVE", seconds, .2, 0, 0)};
            PollenStartDelay bounded = new PollenStartDelay(100, seconds, 27);
            check(seconds + bounded.maximumDelaySeconds() < 26, "Strict transition reserve preserved");
            check(bounded.delaySeconds() == bounded.maximumDelaySeconds(), "Default clamped to legal maximum");
            new PollenSequence(timed, 1250, 1200, 4, .25, 27, bounded.delaySeconds());
        }
        check(new PollenStartDelay(0, 6, 27).maximumDelaySeconds() == 19.5,
                "Exact budget boundary excluded");
        check(new PollenStartDelay(.8, 1.25, 27).delaySeconds() == 1, "Default rounded to delay grid");
        for (double seconds : new double[]{0, -1, 26, Double.NaN, Double.POSITIVE_INFINITY}) {
            boolean rejected = false;
            try { new PollenStartDelay(0, seconds, 27); }
            catch (IllegalArgumentException expected) { rejected = true; }
            check(rejected, "Reject invalid route budget");
        }
        for (double value : new double[]{-1, Double.NaN, Double.POSITIVE_INFINITY}) {
            boolean rejected = false;
            try { new PollenStartDelay(value, 1.25, 27); }
            catch (IllegalArgumentException expected) { rejected = true; }
            check(rejected, "Reject invalid default delay");
        }
        System.out.println("PASS: " + checks + " delay checks (host logic; not hub validation)");
    }
}
