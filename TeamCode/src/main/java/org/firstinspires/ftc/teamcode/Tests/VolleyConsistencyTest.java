package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.methods.BatteryVoltage;
import org.firstinspires.ftc.teamcode.methods.BulkReads;
import org.firstinspires.ftc.teamcode.methods.Intake;
import org.firstinspires.ftc.teamcode.methods.Launcher;
import org.firstinspires.ftc.teamcode.methods.ShootingCoordinator;
import org.firstinspires.ftc.teamcode.methods.SynchronizedFeed;
import org.firstinspires.ftc.teamcode.methods.Transfer;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

/**
 * Four-ball volley test with PER-BALL outcome labels.
 *
 * Uses the production Launcher / Transfer / Intake / ShootingCoordinator with fast-loop
 * settings (MANUAL bulk caching, one shared rate-limited battery reading).
 *
 * Plans (choose in INIT with G1 D-pad RIGHT):
 *   PACED VS CONTINUOUS - 12 volleys, order C P P C C P P C C P P C, transfer 1440.
 *   SPACING SWEEP       - 12 volleys: continuous at 1440 / 1000 / 720 and paced 1440, 3 each.
 *
 * The LED changes color at feed start and at every DETECTED ball contact, so the video
 * shows whether detection matched the real balls:
 *   white = feeding started, red = shot 1, yellow = 2, green = 3, blue = 4, violet = extra.
 *
 * Files go to the same "flywheel-tuning" folder as the stored-batch tests:
 *   volley-consistency-<time>.csv        every loop
 *   volley-consistency-<time>-balls.csv  one row per labeled ball
 *   volley-consistency-<time>-report.txt summary by condition and ball number
 */
@TeleOp(name = "Volley Consistency Test", group = "Diagnostics")
public class VolleyConsistencyTest extends OpMode {

    // ---------- Plans ----------
    private static final class Condition {
        final SynchronizedFeed.Mode mode; final double transfer; final String name;
        Condition(SynchronizedFeed.Mode mode, double transfer) {
            this.mode = mode; this.transfer = transfer;
            name = (mode == SynchronizedFeed.Mode.PACED ? "PACED-" : "CONT-") + (int) transfer;
        }
    }
    private static final Condition C1440 = new Condition(SynchronizedFeed.Mode.CONTINUOUS, 1440);
    private static final Condition C1000 = new Condition(SynchronizedFeed.Mode.CONTINUOUS, 1000);
    private static final Condition C720 = new Condition(SynchronizedFeed.Mode.CONTINUOUS, 720);
    private static final Condition P1440 = new Condition(SynchronizedFeed.Mode.PACED, 1440);

    private static final String[] PLAN_NAMES = {"PACED VS CONTINUOUS", "SPACING SWEEP"};
    private static final Condition[][] PLANS = {
            {C1440, P1440, P1440, C1440, C1440, P1440, P1440, C1440, C1440, P1440, P1440, C1440},
            {C1440, P1440, C720, C1000, C1000, C720, P1440, C1440, C720, C1440, C1000, P1440},
    };
    private int planIndex = 0;

    // ---------- Settings ----------
    private static final int BALLS = 4;
    private double flywheelTarget = 1407;
    /** Auto-end a volley this long after the 4th detected contact. */
    private static final double END_AFTER_LAST_SHOT_S = 0.5;
    private static final double MAX_FEED_S = 6.0;
    private static final double RECOVERED_TOLERANCE = 25;
    private static final double PRE_CONTACT_WINDOW_S = 0.08;

    // LED (goBILDA RGB indicator positions)
    private static final double LED_OFF = 0.0, LED_WHITE = 1.0, LED_VIOLET = 0.722;
    private static final double[] LED_SHOT = {0.277, 0.388, 0.500, 0.611};

    private enum Phase { LOAD, FEEDING, LABEL, COMPLETE }
    private static final String[] LABEL_NAMES = {"HIT", "SHORT", "LONG", "LEFT", "RIGHT", "MISS", "NOT_FIRED"};

    // ---------- Hardware ----------
    private BulkReads bulk;
    private BatteryVoltage battery;
    private Launcher launcher;
    private Transfer transfer;
    private Intake intake;
    private ShootingCoordinator shooting;
    private Servo led;

    // ---------- Volley state ----------
    private Phase phase = Phase.LOAD;
    private int volley = 1, attempt = 1;
    private Condition active;
    private double epoch, lastLoop = Double.NaN, feedStart, volleyEnd, lastFlush;
    private final List<Double> loopDts = new ArrayList<>();
    private final List<Double> allLoopDts = new ArrayList<>();
    // rolling flywheel history for pre-contact speed
    private final double[] histT = new double[64], histF = new double[64];
    private int histN, histHead;
    private final List<Shot> shots = new ArrayList<>();
    private final List<Integer> labels = new ArrayList<>();

    private static final class Shot {
        double time, preSpeed, dipMin, battery, recoverS = Double.NaN, pacedRecoverS = Double.NaN;
    }

    // ---------- Results ----------
    private final int[][] hitsByCondBall = new int[4][BALLS], countByCondBall = new int[4][BALLS];
    private final double[][] preSpeedSum = new double[4][BALLS];
    private final int[][] preSpeedN = new int[4][BALLS];
    private final double[] volleySecondsSum = new double[4];
    private final int[] volleysByCond = new int[4];
    private final List<String> ballRows = new ArrayList<>();

    // ---------- Files ----------
    private BufferedWriter samples, balls;
    private File report;
    private String fileName = "", outputDirectory = "", ioFault = "";
    private final StringBuilder line = new StringBuilder(256);

    @Override public void init() {
        battery = new BatteryVoltage(hardwareMap);
        launcher = new Launcher(hardwareMap, battery);
        transfer = new Transfer(hardwareMap, battery);
        intake = new Intake(hardwareMap, battery);
        shooting = new ShootingCoordinator(launcher, transfer, intake);
        try { led = hardwareMap.get(Servo.class, "led"); } catch (RuntimeException e) { led = null; }
        setLed(LED_OFF);
        bulk = new BulkReads(hardwareMap);
        telemetry.setMsTransmissionInterval(100);
    }

    @Override public void init_loop() {
        bulk.clear();
        if (gamepad1.dpadRightWasPressed()) planIndex = (planIndex + 1) % PLANS.length;
        if (gamepad1.dpadUpWasPressed()) flywheelTarget += 10;
        if (gamepad1.dpadDownWasPressed()) flywheelTarget -= 10;
        flywheelTarget = Math.max(1000, Math.min(1800, flywheelTarget));
        telemetry.addData("Plan (G1 D-pad RIGHT)", PLAN_NAMES[planIndex]);
        telemetry.addData("Order", planOrder());
        telemetry.addData("Flywheel target (G1 D-pad UP/DOWN)", "%.0f ticks/s", flywheelTarget);
        telemetry.addLine("Keep the robot STATIONARY at one marked spot for the whole test.");
        telemetry.addLine("Use a charged battery. Recorder: film the hive AND the robot LED.");
        telemetry.update();
    }

    private String planOrder() {
        StringBuilder sb = new StringBuilder();
        for (Condition c : PLANS[planIndex]) sb.append(c.name).append(' ');
        return sb.toString().trim();
    }

    @Override public void start() {
        epoch = System.nanoTime() / 1e9;
        launcher.setTargetVelocity(flywheelTarget);   // flywheel stays on between volleys
        try {
            File dir = storageDirectory();
            if (!dir.isDirectory() && !dir.mkdirs()) throw new IOException("Cannot create directory");
            outputDirectory = dir.getAbsolutePath();
            fileName = "volley-consistency-" + System.currentTimeMillis();
            report = new File(dir, fileName + "-report.txt");
            samples = new BufferedWriter(new FileWriter(new File(dir, fileName + ".csv")));
            balls = new BufferedWriter(new FileWriter(new File(dir, fileName + "-balls.csv")));
            samples.write("time_s,volley,attempt,condition,phase,feed_state,flywheel_target,flywheel_ticks_s,"
                    + "transfer_target,transfer_ticks_s,intake_ticks_s,battery_V,flywheel_power,transfer_power,"
                    + "flywheel_requested_V,flywheel_integral_V,loop_dt_s,feed_elapsed_s,shot_detected,shot_count\n");
            balls.write("volley,attempt,condition,mode,transfer_setpoint,ball,label,hit,detected,contact_s,"
                    + "gap_from_prev_s,speed_before_contact,dip_min,dip_depth,recover_to_25_s,paced_recover_s,"
                    + "battery_V,volley_s,shots_detected,recover_timeouts,loop_dt_median_ms,loop_dt_p90_ms\n");
            samples.flush(); balls.flush();
            saveReport();
        } catch (IOException e) { storageFault(e); }
    }

    @Override public void loop() {
        bulk.clear();   // FIRST: one bulk read per hub for every encoder this loop
        double now = System.nanoTime() / 1e9 - epoch;
        double dt = now - lastLoop;
        lastLoop = now;
        if (Double.isFinite(dt)) {
            allLoopDts.add(dt);
            if (allLoopDts.size() > 2000) allLoopDts.remove(0);
        }

        // Edge reads: each *WasPressed() is called exactly once per loop.
        boolean a = gamepad2.aWasPressed(), b = gamepad2.bWasPressed(), x = gamepad2.xWasPressed();
        boolean y = gamepad2.yWasPressed(), back = gamepad2.backWasPressed(), lb = gamepad2.leftBumperWasPressed();
        boolean up = gamepad2.dpadUpWasPressed(), down = gamepad2.dpadDownWasPressed();
        boolean left = gamepad2.dpadLeftWasPressed(), right = gamepad2.dpadRightWasPressed();
        boolean abort = gamepad1.bWasPressed();

        Condition[] plan = PLANS[planIndex];
        boolean shoot = false, collect = false, startedThisLoop = false;
        switch (phase) {
            case LOAD:
                collect = gamepad2.left_trigger > .1;
                if (a && ioFault.isEmpty()) { beginVolley(plan[volley - 1], now); startedThisLoop = true; shoot = true; }
                break;
            case FEEDING:
                shoot = true;
                break;
            case LABEL:
                int label = a ? 0 : down ? 1 : up ? 2 : left ? 3 : right ? 4 : b ? 5 : x ? 6 : -1;
                if (label >= 0 && labels.size() < BALLS) labels.add(label);
                if (lb && !labels.isEmpty()) labels.remove(labels.size() - 1);
                if (back) { attempt++; labels.clear(); phase = Phase.LOAD; setLed(LED_OFF); }   // invalid: repeat
                if (y && labels.size() == BALLS) saveVolley(now);
                break;
            default: break;
        }

        shooting.update(shoot, collect, false, true);
        double f = launcher.getMeasuredVelocity();
        remember(now, f);

        if (phase == Phase.FEEDING) {
            if (Double.isFinite(dt)) loopDts.add(dt);
            trackShots(now, f);
            int count = shooting.shotCount();
            double lastShot = shots.isEmpty() ? Double.NaN : shots.get(shots.size() - 1).time;
            boolean done = (count >= BALLS && now - lastShot >= END_AFTER_LAST_SHOT_S)
                    || now - feedStart >= MAX_FEED_S || (a && !startedThisLoop);
            if (abort) { attempt++; endFeed(now); phase = Phase.LOAD; labels.clear(); setLed(LED_OFF); }
            else if (done) { endFeed(now); phase = Phase.LABEL; labels.clear(); }
        }

        writeSample(now, dt, f);
        showTelemetry(f);
    }

    private void beginVolley(Condition c, double now) {
        active = c;
        shooting.setFeedMode(c.mode);
        shooting.setTransferSetpoint(c.transfer);
        launcher.setTargetVelocity(flywheelTarget);
        shots.clear(); loopDts.clear(); labels.clear();
        feedStart = now;
        phase = Phase.FEEDING;
        setLed(LED_WHITE);
    }

    private void endFeed(double now) {
        volleyEnd = now;
        shooting.stopFeed();   // transfer and intake stop; flywheel keeps its target
    }

    private void trackShots(double now, double f) {
        // update dip minimum / recovery for the latest open shot
        if (!shots.isEmpty()) {
            Shot last = shots.get(shots.size() - 1);
            last.dipMin = Math.min(last.dipMin, f);
            if (Double.isNaN(last.recoverS) && f >= flywheelTarget - RECOVERED_TOLERANCE)
                last.recoverS = now - last.time;
        }
        if (shooting.shotDetected()) {
            if (!shots.isEmpty()) {
                Shot prev = shots.get(shots.size() - 1);
                if (Double.isNaN(prev.pacedRecoverS) && active.mode == SynchronizedFeed.Mode.PACED)
                    prev.pacedRecoverS = shooting.lastRecoverSeconds();
            }
            Shot s = new Shot();
            s.time = now; s.dipMin = f; s.battery = battery.get();
            s.preSpeed = maxSince(now - PRE_CONTACT_WINDOW_S);
            shots.add(s);
            int n = shots.size();
            setLed(n <= LED_SHOT.length ? LED_SHOT[n - 1] : LED_VIOLET);
        }
    }

    private void remember(double t, double f) {
        histT[histHead] = t; histF[histHead] = f;
        histHead = (histHead + 1) % histT.length;
        histN = Math.min(histN + 1, histT.length);
    }

    private double maxSince(double t0) {
        double m = Double.NaN;
        for (int i = 0; i < histN; i++) {
            int k = (histHead - 1 - i + histT.length) % histT.length;
            if (histT[k] < t0) break;
            if (Double.isNaN(m) || histF[k] > m) m = histF[k];
        }
        return m;
    }

    private void saveVolley(double now) {
        int cond = conditionIndex(active);
        double volleyS = shots.size() >= 2 ? shots.get(shots.size() - 1).time - shots.get(0).time : Double.NaN;
        double[] lt = percentiles(loopDts);
        try {
            for (int i = 0; i < BALLS; i++) {
                int label = labels.get(i);
                boolean hit = label == 0;
                boolean detected = i < shots.size();
                Shot s = detected ? shots.get(i) : null;
                double gap = detected && i > 0 ? s.time - shots.get(i - 1).time : Double.NaN;
                String row = String.format(Locale.US,
                        "%d,%d,%s,%s,%.0f,%d,%s,%b,%b,%.4f,%.4f,%.1f,%.1f,%.1f,%.4f,%.4f,%.3f,%.4f,%d,%d,%.1f,%.1f",
                        volley, attempt, active.name, active.mode, active.transfer, i + 1, LABEL_NAMES[label], hit, detected,
                        detected ? s.time - feedStart : Double.NaN, gap,
                        detected ? s.preSpeed : Double.NaN, detected ? s.dipMin : Double.NaN,
                        detected ? s.preSpeed - s.dipMin : Double.NaN,
                        detected ? s.recoverS : Double.NaN, detected ? s.pacedRecoverS : Double.NaN,
                        detected ? s.battery : Double.NaN, volleyS, shots.size(), shooting.recoverTimeouts(),
                        lt[0] * 1000, lt[1] * 1000);
                balls.write(row + "\n");
                ballRows.add(row);
                if (cond >= 0) {
                    countByCondBall[cond][i]++;
                    if (hit) hitsByCondBall[cond][i]++;
                    if (detected && Double.isFinite(s.preSpeed)) { preSpeedSum[cond][i] += s.preSpeed; preSpeedN[cond][i]++; }
                }
            }
            balls.flush();
        } catch (IOException e) { storageFault(e); return; }
        if (cond >= 0 && Double.isFinite(volleyS)) { volleySecondsSum[cond] += volleyS; volleysByCond[cond]++; }
        volley++; attempt = 1; labels.clear();
        phase = volley > PLANS[planIndex].length ? Phase.COMPLETE : Phase.LOAD;
        setLed(LED_OFF);
        saveReport();
    }

    private static int conditionIndex(Condition c) {
        return c == C1440 ? 0 : c == P1440 ? 1 : c == C1000 ? 2 : c == C720 ? 3 : -1;
    }
    private static final Condition[] ALL = {C1440, P1440, C1000, C720};

    private static double[] percentiles(List<Double> values) {
        if (values.isEmpty()) return new double[]{Double.NaN, Double.NaN};
        double[] v = new double[values.size()];
        for (int i = 0; i < v.length; i++) v[i] = values.get(i);
        Arrays.sort(v);
        return new double[]{v[v.length / 2], v[Math.min(v.length - 1, (int) (v.length * .9))]};
    }

    private void writeSample(double now, double dt, double f) {
        if (samples == null) return;
        line.setLength(0);
        line.append(fmt(now, 4)).append(',').append(volley).append(',').append(attempt).append(',')
                .append(active == null ? "" : active.name).append(',').append(phase).append(',')
                .append(shooting.state()).append(',').append(fmt(launcher.getTargetVelocity(), 0)).append(',')
                .append(fmt(f, 0)).append(',').append(fmt(transfer.getTargetVelocity(), 0)).append(',')
                .append(fmt(transfer.getMeasuredVelocity(), 0)).append(',').append(fmt(intake.getIntakeVelo(), 0)).append(',')
                .append(fmt(launcher.getBatteryVoltage(), 3)).append(',').append(fmt(launcher.getMotorPower(), 4)).append(',')
                .append(fmt(transfer.getMotorPower(), 4)).append(',').append(fmt(launcher.getRequestedVolts(), 3)).append(',')
                .append(fmt(launcher.getIntegralVolts(), 4)).append(',').append(fmt(dt, 5)).append(',')
                .append(phase == Phase.FEEDING ? fmt(now - feedStart, 4) : "NaN").append(',')
                .append(shooting.shotDetected() ? 1 : 0).append(',').append(shooting.shotCount()).append('\n');
        try {
            samples.write(line.toString());
            if (now - lastFlush >= .5) { samples.flush(); lastFlush = now; }
        } catch (IOException e) { storageFault(e); }
    }

    private static String fmt(double v, int digits) {
        if (!Double.isFinite(v)) return "NaN";
        return String.format(Locale.US, "%." + digits + "f", v);
    }

    private void showTelemetry(double f) {
        Condition[] plan = PLANS[planIndex];
        double[] lt = percentiles(allLoopDts);
        telemetry.addData("Volley", "%d of %d  (attempt %d)", Math.min(volley, plan.length), plan.length, attempt);
        telemetry.addData("Condition", phase == Phase.COMPLETE ? "-" :
                (phase == Phase.LOAD ? plan[volley - 1] : active).name);
        telemetry.addData("Phase / feed state", "%s / %s", phase, shooting.state());
        telemetry.addData("Flywheel", "%.0f / %.0f ticks/s", f, launcher.getTargetVelocity());
        telemetry.addData("Shots detected", shooting.shotCount());
        telemetry.addData("Loop time median / p90", "%.1f / %.1f ms", lt[0] * 1000, lt[1] * 1000);
        switch (phase) {
            case LOAD:
                telemetry.addLine("LOAD 4 balls: hold G2 LEFT TRIGGER. Then G2 A to fire.");
                telemetry.addLine("Recorder: say the volley number out loud before A.");
                break;
            case FEEDING:
                telemetry.addLine("Firing... ends automatically. G2 A ends early; G1 B aborts (repeat).");
                break;
            case LABEL:
                StringBuilder sb = new StringBuilder();
                for (int i = 0; i < BALLS; i++)
                    sb.append(i + 1).append(':').append(i < labels.size() ? LABEL_NAMES[labels.get(i)] : "_").append("  ");
                telemetry.addData("Labels", sb.toString());
                telemetry.addLine("Label ball " + Math.min(labels.size() + 1, BALLS) + " (G2):");
                telemetry.addLine("A=HIT  DOWN=SHORT  UP=LONG  LEFT=LEFT  RIGHT=RIGHT  B=MISS  X=NOT FIRED");
                telemetry.addLine("LB=undo.  Y=save (after 4).  BACK=invalid, repeat volley.");
                break;
            case COMPLETE:
                telemetry.addLine("COMPLETE. Press STOP and send the three files.");
                break;
        }
        for (int c = 0; c < ALL.length; c++) {
            int hits = 0, n = 0;
            for (int i = 0; i < BALLS; i++) { hits += hitsByCondBall[c][i]; n += countByCondBall[c][i]; }
            if (n > 0) telemetry.addData(ALL[c].name + " hits", "%d / %d", hits, n);
        }
        if (!ioFault.isEmpty()) telemetry.addData("STORAGE FAULT", ioFault);
        telemetry.addData("Files", fileName);
        telemetry.addData("Folder", outputDirectory);
        telemetry.update();
    }

    /** Same folder as the stored-batch tests. Overridden only by the desktop simulation. */
    protected File storageDirectory() throws IOException {
        File base = hardwareMap.appContext == null ? null : hardwareMap.appContext.getExternalFilesDir(null);
        if (base == null) throw new IOException("No external storage");
        return new File(base, "flywheel-tuning");
    }

    private void setLed(double position) {
        if (led != null) led.setPosition(position);
    }

    private void storageFault(IOException e) {
        ioFault = e.toString();
        shooting.stopFeed();
        phase = Phase.COMPLETE;
    }

    private void saveReport() {
        if (report == null) return;
        try (BufferedWriter out = new BufferedWriter(new FileWriter(report))) {
            out.write("Volley consistency test: " + PLAN_NAMES[planIndex] + "\n");
            out.write("Order: " + planOrder() + "\n");
            out.write(String.format(Locale.US, "Flywheel target %.0f ticks/s. Paced recovery: +/-%.0f for %.0f ms, max %.0f ms.%n",
                    flywheelTarget, SynchronizedFeed.PACED_RECOVER_TOLERANCE, SynchronizedFeed.PACED_RECOVER_DWELL * 1000,
                    SynchronizedFeed.PACED_MAX_RECOVER_SECONDS * 1000));
            out.write(String.format(Locale.US, "Shot detection: drop >= %.0f below min(peak, target); re-arm after %.0f rise.%n",
                    SynchronizedFeed.SHOT_DROP, SynchronizedFeed.SHOT_REARM_RISE));
            out.write(String.format(Locale.US, "Flywheel KS=%.7f KV=%.9f KP=%.4f KI=%.4f.%n",
                    Launcher.KS, Launcher.KV, Launcher.KP, Launcher.KI));
            double[] lt = percentiles(allLoopDts);
            out.write(String.format(Locale.US, "Loop time (recent) median %.1f ms, p90 %.1f ms.%n%n", lt[0] * 1000, lt[1] * 1000));
            out.write("condition, ball1, ball2, ball3, ball4, total, mean_speed_before_contact_by_ball, mean_volley_s\n");
            for (int c = 0; c < ALL.length; c++) {
                int hits = 0, n = 0;
                StringBuilder perBall = new StringBuilder(), speeds = new StringBuilder();
                for (int i = 0; i < BALLS; i++) {
                    hits += hitsByCondBall[c][i]; n += countByCondBall[c][i];
                    perBall.append(hitsByCondBall[c][i]).append('/').append(countByCondBall[c][i]).append(", ");
                    speeds.append(preSpeedN[c][i] == 0 ? "NaN" : fmt(preSpeedSum[c][i] / preSpeedN[c][i], 0)).append(' ');
                }
                if (n == 0) continue;
                out.write(ALL[c].name + ", " + perBall + hits + "/" + n + ", " + speeds.toString().trim() + ", "
                        + (volleysByCond[c] == 0 ? "NaN" : fmt(volleySecondsSum[c] / volleysByCond[c], 2)) + "\n");
            }
            out.write("\nPer-ball rows:\n");
            for (String row : ballRows) out.write(row + "\n");
        } catch (IOException e) { ioFault = e.toString(); }
    }

    @Override public void stop() {
        if (shooting != null) shooting.stopAll();
        setLed(LED_OFF);
        saveReport();
        for (BufferedWriter w : new BufferedWriter[]{samples, balls})
            if (w != null) try { w.close(); } catch (IOException ignored) { }
    }
}
