import org.firstinspires.ftc.teamcode.pollen.PollenSequence;
import org.firstinspires.ftc.teamcode.pollen.PollenShootTuning;
import static org.firstinspires.ftc.teamcode.pollen.PollenShootTuning.Field.*;

/** Behavioral checks for the new shoot-first mode; no FTC/Android dependencies. */
public final class PollenShootFirstTest {
    private static int checks;
    private static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }
    private static void near(double got, double expected, String message) {
        check(Math.abs(got - expected) < .000001, message + ": " + got + " != " + expected);
    }
    private static void zero(PollenSequence.Output out) {
        check(out.forward == 0 && out.strafe == 0 && out.turn == 0
                && out.launcherVelocity == 0 && out.feederPower == 0, "zero outputs");
    }
    private static void release(PollenShootTuning t) { t.update(false, false, false, false, false); }
    private static void select(PollenShootTuning t, PollenShootTuning.Field field) {
        while (t.selected() != field) {
            t.update(false, true, false, false, false);
            release(t);
        }
    }
    private static void change(PollenShootTuning t, int direction) {
        t.update(false, false, direction < 0, direction > 0, false);
        release(t);
    }
    public static void main(String[] args) {
        PollenShootTuning t = new PollenShootTuning(null);
        near(t.value(DISTANCE_CM), 60.96, "two-foot default");
        near(t.driveSeconds(), 2.3627906976744186, "two-foot time with measured diagonal factor");
        near(t.value(FEED_SECONDS), 10, "ten seconds excludes spinup");
        near(t.value(FEED_POWER), .5, "both feeds match neutral-bumper TeleOp command");
        near(t.forward() * t.forward() + t.strafe() * t.strafe(), .04, "vector magnitude");
        check(t.forward() > 0 && t.strafe() < 0, "forward-left departure");
        near(t.forward(), .20 * Math.cos(Math.toRadians(25.87)), "fixed forward component");
        near(t.strafe(), -.20 * Math.sin(Math.toRadians(25.87)), "fixed left component");

        PollenSequence s = t.sequence();
        zero(s.tick(0, 0, false));
        s.start(0);
        PollenSequence.Output spin = s.tick(0, 0, false);
        near(spin.launcherVelocity, 1250, "flywheel starts alone");
        near(spin.feederPower, 0, "no premature feed");
        near(spin.forward, 0, "no approach drive");
        near(s.tick(1, 1199, false).feederPower, 0, "wait for target band");
        s.tick(1.25, 1250, false);
        near(s.tick(1.49, 1250, false).feederPower, 0, "quarter second stability");
        near(s.tick(1.5, 1250, false).feederPower, 1, "begin continuous feed");
        PollenSequence.Output dip = s.tick(2, 900, false);
        near(dip.feederPower, 1, "ordinary speed dip does not pause continuous feed");
        near(dip.launcherVelocity, 1250, "flywheel target maintained during feed");
        near(dip.forward, 0, "stationary shooting");
        near(s.tick(11.49, 1250, false).feederPower, 1, "full ten seconds after qualification");
        zero(s.tick(11.5, 1250, false));
        PollenSequence.Output move = s.tick(11.51, 0, false);
        near(move.forward, t.forward(), "depart after feeding");
        near(move.strafe, t.strafe(), "diagonal departure");
        near(move.turn, 0, "does not rotate");
        near(move.launcherVelocity, 0, "flywheel off during travel");
        near(move.feederPower, 0, "both feeds off during travel");
        for (double p : PollenSequence.wheelPowers(move)) check(Math.abs(p) <= 1, "bounded wheel power");
        double travelEnds = 11.5 + t.driveSeconds();
        zero(s.tick(travelEnds + .001, 0, false));
        PollenSequence.Output done = s.tick(travelEnds + .502, 0, false);
        check(done.finished, "hold then finish"); zero(done);

        s = t.sequence(); s.start(0);
        s.tick(0, 0, false); zero(s.tick(4, 0, false));
        check(s.warning().contains("not ready"), "spinup timeout reported");
        near(s.tick(4.01, 0, false).strafe, t.strafe(), "timeout uses selected diagonal, not Park A");
        s = t.sequence(false); s.start(0);
        near(s.tick(0, 0, false).strafe, t.strafe(), "INIT fallback uses same departure");
        for (double stopAt : new double[]{0, .5, 3, 11.5, 12}) {
            s = t.sequence(); s.start(0);
            for (double now = 0; now < stopAt; now += .02) s.tick(now, 1250, false);
            zero(s.tick(stopAt, 1250, true));
            PollenSequence.Output stopped = s.tick(stopAt + 1, 1250, false);
            zero(stopped); check(stopped.finished, "STOP latches in every phase");
        }
        s = t.sequence(); s.start(0); s.tick(0, 1250, false); s.tick(.25, 1250, false);
        check(s.tick(.5, Double.NaN, false).finished, "invalid encoder data stops all");
        s = t.sequence(); s.start(0); s.tick(1, 1250, false);
        check(s.tick(.5, 1250, false).finished, "backward clock stops");
        s = t.sequence(); s.start(0);
        check(s.tick(27, 1250, false).finished, "independent deadline");

        // UI edges, limits, budget rejection, session memory and locking.
        select(t, FLYWHEEL);
        change(t, 1); near(t.value(FLYWHEEL), 1300, "speed adjustment");
        t.update(false, false, false, true, false);
        t.update(false, false, false, true, false);
        near(t.value(FLYWHEEL), 1350, "held key does not repeat"); release(t);
        t.update(false, false, true, true, false);
        near(t.value(FLYWHEEL), 1350, "opposite arrows cancel"); release(t);
        select(t, START_DELAY); change(t, 1);
        PollenShootTuning remembered = new PollenShootTuning(t.snapshot());
        near(remembered.value(FLYWHEEL), 1350, "remember speed");
        near(remembered.value(START_DELAY), 0, "reset delay on INIT");
        near(t.value(START_DELAY), .5, "snapshots do not alias");
        for (int i = 0; i < 60; i++) change(t, 1);
        check(t.worstCaseSeconds() < 26, "budget bounded");
        check(t.notice().contains("rejected"), "budget rejection visible");
        s = t.sequence(); s.start(0); zero(s.tick(t.value(START_DELAY) - .01, 0, false));
        near(s.tick(t.value(START_DELAY), 0, false).launcherVelocity, 1350, "delay precedes spinup");
        t.lock(); double before = t.value(START_DELAY);
        change(t, -1); t.update(false, false, false, false, true);
        near(t.value(START_DELAY), before, "START locks edits and reset");

        t = new PollenShootTuning(null);
        select(t, FEED_SECONDS); for (int i = 0; i < 25; i++) change(t, -1);
        near(t.value(FEED_SECONDS), 0, "skip shot for motion calibration");
        s = t.sequence(); s.start(0); near(s.tick(0, 0, false).forward, t.forward(), "skip-shot route");
        select(t, DISTANCE_CM); for (int i = 0; i < 100; i++) change(t, 1);
        near(t.value(DISTANCE_CM), 350, "distance upper bound");
        for (int i = 0; i < 100; i++) change(t, -1);
        near(t.value(DISTANCE_CM), 0, "zero distance for stationary test");
        s = t.sequence(); s.start(0); zero(s.tick(0, 0, false));
        check(s.tick(.5, 0, false).finished, "both zero gives hold only");
        t.update(false, false, false, false, true); release(t);
        near(t.value(DISTANCE_CM), 60.96, "triangle restores short test");
        near(t.value(FEED_SECONDS), 10, "triangle restores feed duration");
        select(t, DISTANCE_CM); for (int i = 0; i < 44; i++) change(t, 1);
        check(t.worstCaseSeconds() < 26, "nominal field-sized route fits default budget");
        select(t, DRIVE_POWER); for (int i = 0; i < 10; i++) change(t, -1);
        check(t.worstCaseSeconds() < 26, "slowing drive cannot silently exhaust budget");
        t.sequence(); // Still a valid sequence after refused changes.
        System.out.println("PollenShootFirstTest: " + checks + " checks passed");
    }
}
