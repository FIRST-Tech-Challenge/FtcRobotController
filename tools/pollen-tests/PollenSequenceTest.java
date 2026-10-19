import org.firstinspires.ftc.teamcode.pollen.PollenConfig;
import org.firstinspires.ftc.teamcode.pollen.PollenSequence;

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
        return sequence(new PollenSequence.Step[]{PollenSequence.Step.shoot(2),
                PollenSequence.Step.drive("PARK", 1, 0.2, 0, 0)});
    }
    public static void main(String[] args) {
        PollenSequence park = sequence(PollenConfig.parkRoute());
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
        check(shot.tick(0.1, 1250, false).feederPower == 0, "Require stable speed");
        check(shot.tick(0.2, 0, false).feederPower == 0, "Reset stability on dip");
        shot.tick(0.3, 1250, false);
        check(shot.tick(0.56, 1250, false).feederPower == 1, "Feed after stable speed");
        check(shot.tick(0.6, 1100, false).feederPower == 0, "Pause feeding on low speed");
        check(shot.tick(0.7, Double.NaN, false).feederPower == 0, "No feed on invalid velocity");
        check(shot.tick(0.8, 1250, false).feederPower == 1, "Resume on recovery");
        zero(shot.tick(2.57, 1250, false));
        check(shot.tick(2.58, 0, false).forward == 0.2, "Park after shooting");
        check(shot.tick(3.58, 0, false).finished, "Shot route completes");

        PollenSequence timeout = shot(); timeout.start(0);
        timeout.tick(0, 0, false);
        zero(timeout.tick(4, 0, false));
        check(!timeout.warning().isEmpty(), "Timeout reported");
        check(timeout.tick(4.01, 0, false).forward == 0.2, "Park after launcher timeout");

        // Stop before start, during driving, spinup, feeding, and parking must latch zero.
        for (double stopAt : new double[]{0, 0.5, 1.5, 2.8, 4.5, 6.0}) {
            PollenSequence s = sequence(PollenConfig.shootRoute()); s.start(0);
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
                PollenSequence s = sequence(shoot ? PollenConfig.shootRoute() : PollenConfig.parkRoute());
                s.start(0); boolean finished = false; boolean fed = false;
                for (double t = 0; t < 15; t += .02) {
                    PollenSequence.Output out = s.tick(t, failure ? 0 : 1250, false);
                    fed |= out.feederPower > 0;
                    if (out.finished) { zero(out); finished = true; break; }
                }
                check(finished, "Full route finishes");
                check(fed == (shoot && !failure), "Expected feed outcome");
            }
        }
        check(!PollenConfig.ENABLE_ROBOT_PARK_TEST && !PollenConfig.ENABLE_ROBOT_SHOOT_TEST,
                "Distributed code keeps physical robot tests disabled");
        System.out.println("PASS: " + checks + " checks (pure Java sequence; not Android/hardware validation)");
    }
}
