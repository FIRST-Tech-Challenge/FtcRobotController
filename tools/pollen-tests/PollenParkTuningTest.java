import org.firstinspires.ftc.teamcode.pollen.PollenConfig;
import org.firstinspires.ftc.teamcode.pollen.PollenParkTuning;
import org.firstinspires.ftc.teamcode.pollen.PollenSequence;

public final class PollenParkTuningTest {
    private static int checks;
    private static void check(boolean value, String message) {
        checks++;
        if (!value) throw new AssertionError(message);
    }
    private static void equal(double actual, double expected, String message) {
        check(Math.abs(actual - expected) < 0.000001, message + ": " + actual);
    }
    private static PollenParkTuning create(double deadline) {
        return new PollenParkTuning(0.20, 0.75, 0, deadline, 0.5, null);
    }
    private static void release(PollenParkTuning t) {
        t.update(false, false, false, false, false, false, false);
    }
    private static void press(PollenParkTuning t, int key) {
        release(t);
        t.update(key == 0, key == 1, key == 2, key == 3, key == 4, key == 5, key == 6);
    }
    public static void main(String[] args) {
        PollenParkTuning t = create(27);
        equal(t.power(), 0.2, "Default power");
        equal(t.seconds(), 0.75, "Default time");
        press(t, 3);
        equal(t.seconds(), 0.80, "Right increases time");
        t.update(false, false, false, true, false, false, false);
        equal(t.seconds(), 0.80, "Held button does not repeat");
        press(t, 5);
        equal(t.power(), 0.25, "R1 increases power");
        press(t, 0);
        equal(t.delaySeconds(), 0.5, "Up increases delay");
        press(t, 2); press(t, 4); press(t, 1);
        equal(t.seconds(), 0.75, "Left decreases time");
        equal(t.power(), 0.20, "L1 decreases power");
        equal(t.delaySeconds(), 0, "Down decreases delay");

        release(t);
        t.update(true, true, true, true, true, true, false);
        equal(t.power(), 0.20, "Opposing power buttons ignored");
        equal(t.seconds(), 0.75, "Opposing time buttons ignored");
        equal(t.delaySeconds(), 0, "Opposing delay buttons ignored");
        for (int i = 0; i < 150; i++) { press(t, 5); press(t, 3); press(t, 0); }
        equal(t.power(), 0.50, "Power ceiling");
        equal(t.seconds(), 5, "Time ceiling");
        equal(t.delaySeconds(), 20, "Delay ceiling");
        check(t.seconds() + 0.5 + t.delaySeconds() < 26, "Reserve maintained at all maxima");
        for (int i = 0; i < 150; i++) { press(t, 4); press(t, 2); press(t, 1); }
        equal(t.power(), 0.05, "Power floor");
        equal(t.seconds(), 0.05, "Time floor");
        equal(t.delaySeconds(), 0, "Delay floor");
        release(t);
        t.update(true, false, false, true, false, true, true);
        equal(t.power(), 0.20, "Triangle reset dominates power");
        equal(t.seconds(), 0.75, "Triangle reset dominates time");
        equal(t.delaySeconds(), 0, "Triangle reset dominates delay");

        press(t, 3); press(t, 5); press(t, 0);
        PollenParkTuning next = new PollenParkTuning(0.20, 0.75, 0, 27, 0.5, t.snapshot());
        equal(next.seconds(), 0.80, "Remembered time");
        equal(next.power(), 0.25, "Remembered power");
        equal(next.delaySeconds(), 0, "New INIT resets delay");
        check(!next.locked(), "New INIT allows editing");
        t.lock();
        press(t, 6); press(t, 4); press(t, 2); press(t, 1);
        equal(t.seconds(), 0.80, "Time frozen after START");
        equal(t.power(), 0.25, "Power frozen after START");
        equal(t.delaySeconds(), 0.5, "Delay frozen after START");

        PollenParkTuning shortBudget = create(10);
        for (int i = 0; i < 50; i++) press(shortBudget, 0);
        equal(shortBudget.delaySeconds(), 7.5, "Short budget caps delay");
        for (int i = 0; i < 100; i++) press(shortBudget, 3);
        equal(shortBudget.delaySeconds(), 3, "Longer drive automatically reduces delay");
        check(shortBudget.seconds() + 0.5 + shortBudget.delaySeconds() < 9, "Strict deadline after edit");
        boolean rejected = false;
        try { create(6); } catch (IllegalArgumentException expected) { rejected = true; }
        check(rejected, "Impossible tuning budget rejected");
        rejected = false;
        try { new PollenParkTuning(Double.NaN, 0.75, 0, 27, 0.5, null); }
        catch (IllegalArgumentException expected) { rejected = true; }
        check(rejected, "Invalid default rejected");

        PollenSequence sequence = PollenSequence.park(PollenConfig.parkRoute(t.seconds(), t.power()),
                27, t.delaySeconds());
        sequence.start(0);
        equal(sequence.tick(0.49, 0, false).forward, 0, "No output during selected delay");
        equal(sequence.tick(0.5, 0, false).forward, 0.25, "Route uses selected power");
        equal(sequence.tick(1.29, 0, false).forward, 0.25, "Route uses selected time");
        equal(sequence.tick(1.31, 0, false).forward, 0, "Drive stops after selected time");
        sequence.stop("Operator stop");
        equal(sequence.tick(2, 0, false).forward, 0, "STOP remains latched");
        System.out.println("PASS: " + checks + " parking-tuning checks (host logic, not robot calibration)");
    }
}
