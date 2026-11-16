package org.firstinspires.ftc.teamcode.methods;

import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Paced vs continuous feeding against a crude tunnel/flywheel model built from the
 * stored-batch logs: ~120 ticks/s dip per ball, ~650 ticks/s^2 recovery, balls ~274
 * transfer-ticks apart, SDK velocity quantized to 20 ticks/s. The model checks the
 * state machine's intent; it is not a physics validation.
 */
public final class PacedFeedRegression {
    static final double TARGET = 1407, DT = .010;

    static final class Sim {
        final SynchronizedFeed feed = new SynchronizedFeed();
        double t, f = TARGET, transferMoved;
        final List<Double> ballFronts = new ArrayList<>();
        final List<Double> contactSpeeds = new ArrayList<>();
        final List<Double> contactTimes = new ArrayList<>();
        double dipRemaining;
        boolean feedingDuringRecover;
        double recoverRate = 650, noise = 0;
        int detected;
        java.util.Random random = new java.util.Random(7);

        Sim(SynchronizedFeed.Mode mode, double transferSetpoint) {
            feed.setMode(mode);
            feed.setTransferSetpoint(transferSetpoint);
            for (int i = 0; i < 4; i++) ballFronts.add(150.0 + 274.0 * i);
        }

        double measured() {
            double n = noise == 0 ? 0 : (random.nextDouble() * 2 - 1) * noise;
            return Math.round((f + n) / 20.0) * 20.0;
        }

        void step(boolean request) {
            t += DT;
            feed.update(t, request, true, TARGET, measured(), 0, true);
            if (feed.shotDetected()) detected++;
            double transfer = feed.transferTarget();
            if (feed.state() == SynchronizedFeed.State.RECOVER && (transfer > 0 || feed.intakeEnabled()))
                feedingDuringRecover = true;
            // Balls advance with the commanded transfer speed (ideal motor).
            double move = transfer * DT;
            for (int i = 0; i < ballFronts.size(); i++) ballFronts.set(i, ballFronts.get(i) - move);
            if (!ballFronts.isEmpty() && ballFronts.get(0) <= 0) {
                contactSpeeds.add(f); contactTimes.add(t);
                ballFronts.remove(0);
                dipRemaining += 120;
            }
            // Contact removes energy over ~30 ms; the motor recovers otherwise.
            if (dipRemaining > 0) {
                double d = Math.min(dipRemaining, 120 * DT / .030);
                f -= d; dipRemaining -= d;
            } else if (f < TARGET) {
                f = Math.min(TARGET, f + recoverRate * DT);
            }
        }
    }

    public static void main(String[] args) {
        // 1. Continuous: original behavior, later balls meet a slower wheel.
        Sim c = new Sim(SynchronizedFeed.Mode.CONTINUOUS, 1440);
        for (int i = 0; i < 300; i++) c.step(true);
        check(c.contactSpeeds.size() == 4, "Continuous fires all four balls: " + c.contactSpeeds);
        check(c.detected == 4, "Continuous detects four contacts, got " + c.detected);
        check(c.contactSpeeds.get(0) >= TARGET - 25, "Ball 1 meets a ready wheel");
        double worstLater = Collections.min(c.contactSpeeds.subList(1, 4));
        check(worstLater < TARGET - 40, "Model reproduces the logged underspeed for balls 2-4: " + worstLater);

        // 2. Paced: every ball meets a recovered wheel; feed motors stay off while recovering.
        Sim p = new Sim(SynchronizedFeed.Mode.PACED, 1440);
        for (int i = 0; i < 500; i++) p.step(true);
        check(p.contactSpeeds.size() == 4, "Paced fires all four balls: " + p.contactSpeeds);
        check(p.detected == 4 && p.feed.shotCount() == 4, "Paced detects four contacts, got " + p.detected);
        for (int i = 0; i < 4; i++)
            check(p.contactSpeeds.get(i) >= TARGET - 30,
                    "Paced ball " + (i + 1) + " meets a recovered wheel: " + p.contactSpeeds.get(i));
        check(!p.feedingDuringRecover, "Transfer and intake are off during every recovery");
        check(p.feed.recoverTimeouts() == 0, "No recovery timeouts with a healthy wheel");
        double pacedVolley = p.contactTimes.get(3) - p.contactTimes.get(0);
        double contVolley = c.contactTimes.get(3) - c.contactTimes.get(0);
        check(pacedVolley > contVolley && pacedVolley < 2.0,
                "Paced volley is slower but under 2 s: " + pacedVolley + " vs " + contVolley);

        // 3. Steady-state noise and quantization never fake a shot (5 s, no balls reach the wheel).
        Sim n = new Sim(SynchronizedFeed.Mode.PACED, 1440);
        n.ballFronts.clear(); n.noise = 25;
        for (int i = 0; i < 500; i++) n.step(true);
        check(n.detected == 0, "No false shots from +/-25 noise, got " + n.detected);

        // 4. A single high outlier sample cannot set up a false dip.
        SynchronizedFeed o = new SynchronizedFeed();
        o.setMode(SynchronizedFeed.Mode.PACED);
        double time = 0;
        double[] seq = {1400, 1420, 1520, 1420, 1400, 1420, 1400, 1400};
        for (double v : seq) { time += DT; o.update(time, true, true, TARGET, v, 0, true); }
        check(o.shotCount() == 0, "High outlier does not cause a false shot");

        // 5. A wheel that never recovers resumes after the max time and records a timeout.
        Sim s = new Sim(SynchronizedFeed.Mode.PACED, 1440);
        s.feed.setPacedRecovery(25, .05, .30);
        s.recoverRate = 0;
        for (int i = 0; i < 200; i++) s.step(true);
        check(s.feed.recoverTimeouts() >= 1, "Stalled recovery times out instead of hanging");
        check(s.contactSpeeds.size() >= 2, "Feeding resumes after a timeout");

        // 6. Settings cannot change mid-batch; releasing the request returns to IDLE.
        boolean threw = false;
        try { p.feed.setMode(SynchronizedFeed.Mode.CONTINUOUS); } catch (IllegalStateException e) { threw = true; }
        check(threw, "Mode change rejected while active");
        p.step(false);
        check(p.feed.state() == SynchronizedFeed.State.IDLE && p.feed.transferTarget() == 0 && !p.feed.intakeEnabled(),
                "Releasing the request stops feeding");
        p.feed.setMode(SynchronizedFeed.Mode.CONTINUOUS);

        // 7. Slower transfer spaces balls further apart in continuous mode.
        Sim slow = new Sim(SynchronizedFeed.Mode.CONTINUOUS, 720);
        for (int i = 0; i < 400; i++) slow.step(true);
        check(slow.contactSpeeds.size() == 4, "Slow continuous fires all four balls");
        check(slow.contactTimes.get(1) - slow.contactTimes.get(0) > 1.8 * (c.contactTimes.get(1) - c.contactTimes.get(0)),
                "Half transfer speed roughly doubles ball spacing");

        // 8. BatteryVoltage reads hardware at most every interval and keeps the last good value.
        final double[] volts = {12.5};
        final int[] hw = {0};
        VoltageSensor sensor = (VoltageSensor) Proxy.newProxyInstance(VoltageSensor.class.getClassLoader(),
                new Class<?>[]{VoltageSensor.class}, (proxy, method, a) -> {
                    if (method.getName().equals("getVoltage")) { hw[0]++; return volts[0]; }
                    throw new AssertionError(method.getName());
                });
        final double[] clock = {0};
        BatteryVoltage battery = new BatteryVoltage(Collections.singletonList(sensor), () -> clock[0]);
        for (int i = 0; i < 100; i++) { clock[0] = i * .010; battery.get(); }   // 1 s at 100 Hz
        check(hw[0] >= 9 && hw[0] <= 11, "About 10 hardware reads per second, got " + hw[0]);
        volts[0] = Double.NaN; clock[0] += .2;
        check(battery.get() == 12.5, "Invalid reading keeps last good voltage");
        volts[0] = 11.9; clock[0] += .2;
        check(battery.get() == 11.9, "New valid reading is used");

        System.out.printf("Model contact speeds  continuous %s (volley %.2f s)%n", c.contactSpeeds, contVolley);
        System.out.printf("Model contact speeds  paced      %s (volley %.2f s)%n", p.contactSpeeds, pacedVolley);
        System.out.println("Paced feed, shot detection, timeout, spacing and battery-throttle checks passed");
    }

    static void check(boolean ok, String message) {
        if (!ok) throw new AssertionError(message);
    }
}
