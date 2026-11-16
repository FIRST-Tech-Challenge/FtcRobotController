package org.firstinspires.ftc.teamcode.methods;

import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Tests.VolleyConsistencyTest;

import java.io.File;
import java.lang.reflect.Field;
import java.lang.reflect.Proxy;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Runs the real VolleyConsistencyTest OpMode end to end (12 volleys, labels, files)
 * against simulated motors, LED and gamepads in real time (~10 ms loops).
 * Physics are a crude first-order flywheel plus a four-ball tunnel; this checks the
 * OpMode's flow, logging and LED cues, not the robot's real behavior.
 */
public final class VolleyConsistencySim {
    static void check(boolean ok, String message) { if (!ok) throw new AssertionError(message); }

    static final class Map2 extends HardwareMap {
        final Map<String, HardwareDevice> devices = new HashMap<>();
        Map2() { super(null, null); }
        void add(String name, HardwareDevice d) { devices.put(name, d); super.put(name, d); }
        @Override public <T> T get(Class<? extends T> type, String name) {
            HardwareDevice d = devices.get(name);
            if (d == null) throw new IllegalArgumentException("No device " + name);
            return type.cast(d);
        }
    }

    // ----- simulated physics -----
    static double battery = 12.6, fPower, tPower, iPower, f;   // f = true flywheel ticks/s
    static double dipRemaining;
    static final List<Double> balls = new ArrayList<>();
    static final List<Double> contactSpeeds = new ArrayList<>();
    static final List<Double> ledPositions = new ArrayList<>();

    static double transferSpeed() {
        return Math.max(0, (tPower * battery - Transfer.KS) / Transfer.KV);
    }

    static void step(double dt) {
        if (dt <= 0 || dt > .1) return;
        double move = transferSpeed() * dt;
        for (int i = 0; i < balls.size(); i++) balls.set(i, balls.get(i) - move);
        if (!balls.isEmpty() && balls.get(0) <= 0) {
            contactSpeeds.add(f); balls.remove(0); dipRemaining += 120;
        }
        if (dipRemaining > 0) {
            double d = Math.min(dipRemaining, 120 * dt / .030); f -= d; dipRemaining -= d;
        }
        double volts = fPower * battery;
        f += (volts - Launcher.KS - Launcher.KV * f) / (Launcher.KV * 1.3) * dt;
        f = Math.max(0, f);
    }

    static DcMotorEx motor(String name, java.util.function.DoubleSupplier velocity, java.util.function.DoubleConsumer power) {
        final DcMotor.RunMode[] mode = {DcMotor.RunMode.RUN_WITHOUT_ENCODER};
        return (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(), new Class<?>[]{DcMotorEx.class},
                (proxy, m, a) -> {
                    switch (m.getName()) {
                        case "setMode": mode[0] = (DcMotor.RunMode) a[0]; return null;
                        case "getMode": return mode[0];
                        case "setDirection": case "setZeroPowerBehavior": return null;
                        case "setPower": power.accept((double) a[0]); return null;
                        case "getPower": return 0.0;
                        case "getVelocity": return velocity.getAsDouble();
                        case "getCurrentPosition": return 0;
                        case "getDeviceName": case "getConnectionInfo": return name;
                        case "getManufacturer": return HardwareDevice.Manufacturer.Other;
                        case "hashCode": return System.identityHashCode(proxy);
                        case "equals": return proxy == a[0];
                        default: throw new AssertionError("Unexpected motor API: " + m);
                    }
                });
    }

    static Servo servo(String name, java.util.function.DoubleConsumer position) {
        return (Servo) Proxy.newProxyInstance(Servo.class.getClassLoader(), new Class<?>[]{Servo.class},
                (proxy, m, a) -> {
                    switch (m.getName()) {
                        case "setPosition": position.accept((double) a[0]); return null;
                        case "getPosition": return 0.0;
                        case "getDeviceName": case "getConnectionInfo": return name;
                        case "getManufacturer": return HardwareDevice.Manufacturer.Other;
                        case "hashCode": return System.identityHashCode(proxy);
                        case "equals": return proxy == a[0];
                        default: throw new AssertionError("Unexpected servo API: " + m);
                    }
                });
    }

    static Object defaultFor(Class<?> t) {
        if (t == boolean.class) return false;
        if (t == int.class) return 0;
        if (t == long.class) return 0L;
        if (t == double.class) return 0.0;
        return null;
    }

    static final class Sim extends VolleyConsistencyTest {
        final File dir;
        Sim(File dir) { this.dir = dir; }
        @Override protected File storageDirectory() { return dir; }
    }

    static String phase(VolleyConsistencyTest op) throws Exception {
        Field fld = VolleyConsistencyTest.class.getDeclaredField("phase");
        fld.setAccessible(true);
        return String.valueOf(fld.get(op));
    }

    static final Gamepad pad1 = new Gamepad(), pad2 = new Gamepad();

    static long lastNs;
    static void tick(VolleyConsistencyTest op) throws Exception {
        long now = System.nanoTime();
        step((now - lastNs) / 1e9);
        lastNs = now;
        op.gamepad1.copy(pad1);
        op.gamepad2.copy(pad2);
        op.loop();
        Thread.sleep(10);
    }

    /** Press a G2 button for one loop, then release it for one loop. */
    static void press(VolleyConsistencyTest op, String button) throws Exception {
        Field b = Gamepad.class.getField(button);
        b.setBoolean(pad2, true); tick(op);
        b.setBoolean(pad2, false); tick(op);
    }

    public static void main(String[] args) throws Exception {
        File dir = Files.createTempDirectory("volley-sim").toFile();
        Map2 map = new Map2();
        map.add("fWheel", motor("fWheel", () -> -Math.round(f / 20.0) * 20.0, p -> fPower = p));
        map.add("LaunchMech", motor("LaunchMech", () -> Math.round(transferSpeed() / 20.0) * 20.0, p -> tPower = p));
        map.add("Intake", motor("Intake", () -> 0.0, p -> iPower = p));
        map.add("Ramp", servo("Ramp", p -> { }));
        map.add("led", servo("led", ledPositions::add));
        VoltageSensor v = (VoltageSensor) Proxy.newProxyInstance(VoltageSensor.class.getClassLoader(),
                new Class<?>[]{VoltageSensor.class}, (proxy, m, a) -> {
                    switch (m.getName()) {
                        case "getVoltage": return battery;
                        case "getDeviceName": case "getConnectionInfo": return "battery";
                        case "getManufacturer": return HardwareDevice.Manufacturer.Other;
                        case "hashCode": return System.identityHashCode(proxy);
                        case "equals": return proxy == a[0];
                        default: throw new AssertionError("Unexpected voltage API: " + m);
                    }
                });
        map.add("battery", v);
        map.voltageSensor.put("battery", v);

        Sim op = new Sim(dir);
        op.hardwareMap = map;
        op.gamepad1 = new Gamepad();
        op.gamepad2 = new Gamepad();
        op.telemetry = (Telemetry) Proxy.newProxyInstance(Telemetry.class.getClassLoader(),
                new Class<?>[]{Telemetry.class}, (proxy, m, a) -> defaultFor(m.getReturnType()));

        op.init();
        for (int i = 0; i < 3; i++) { op.init_loop(); }
        op.start();
        lastNs = System.nanoTime();
        f = 0;
        // Spin up
        for (int i = 0; i < 300 && f < 1380; i++) tick(op);
        check(f > 1350, "Flywheel spins up in simulation: " + f);

        int volleys = 0;
        List<List<Double>> perVolley = new ArrayList<>();
        while (!phase(op).equals("COMPLETE") && volleys < 20) {
            check(phase(op).equals("LOAD"), "Expected LOAD before volley, got " + phase(op));
            balls.clear();
            for (int i = 0; i < 4; i++) balls.add(80.0 + 274.0 * i);   // tightly packed tunnel
            int before = contactSpeeds.size();
            press(op, "a");
            int loops = 0;
            while (phase(op).equals("FEEDING") && loops++ < 800) tick(op);
            check(phase(op).equals("LABEL"), "Volley ends by itself and asks for labels, got " + phase(op));
            perVolley.add(new ArrayList<>(contactSpeeds.subList(before, contactSpeeds.size())));
            press(op, "a"); press(op, "dpad_down"); press(op, "a");
            press(op, "b"); press(op, "left_bumper");                     // undo the MISS
            press(op, "dpad_right");
            press(op, "y");
            volleys++;
        }
        check(volleys == 12, "Default plan runs 12 volleys, ran " + volleys);
        op.stop();

        File[] files = dir.listFiles();
        check(files != null && files.length == 3, "Three output files written");
        File ballsCsv = null, report = null, samples = null;
        for (File file : files) {
            if (file.getName().endsWith("-balls.csv")) ballsCsv = file;
            else if (file.getName().endsWith("-report.txt")) report = file;
            else samples = file;
        }
        List<String> rows = Files.readAllLines(ballsCsv.toPath());
        check(rows.size() == 1 + 48, "48 labeled ball rows, got " + (rows.size() - 1));
        check(rows.get(1).contains(",HIT,") && rows.get(2).contains(",SHORT,") && rows.get(4).contains(",RIGHT,"),
                "Labels recorded in order with undo applied: " + rows.get(4));
        int undetected = 0;
        for (String r : rows.subList(1, rows.size())) if (r.split(",")[8].equals("false")) undetected++;
        check(undetected == 0, "Every simulated ball was detected, missing " + undetected);
        String rep = new String(Files.readAllBytes(report.toPath()));
        check(rep.contains("CONT-1440, 6/6, 0/6, 6/6, 0/6, 12/24") && rep.contains("PACED-1440, 6/6, 0/6, 6/6, 0/6, 12/24"),
                "Report summarises both conditions by ball:\n" + rep);
        List<String> sampleRows = Files.readAllLines(samples.toPath());
        check(sampleRows.size() > 500, "Per-loop CSV written: " + sampleRows.size());
        check(ledPositions.contains(1.0) && ledPositions.contains(0.277) && ledPositions.contains(0.611),
                "LED shows feed start and shot colors");

        // Paced volleys (plan positions 2,3,6,7,10,11) should meet a recovered wheel on every ball.
        int[] pacedIdx = {1, 2, 5, 6, 9, 10};
        double worstPaced = Double.POSITIVE_INFINITY, worstCont = Double.POSITIVE_INFINITY;
        for (int i = 0; i < perVolley.size(); i++) {
            boolean paced = false;
            for (int p : pacedIdx) paced |= p == i;
            for (int k = 1; k < perVolley.get(i).size(); k++) {
                if (paced) worstPaced = Math.min(worstPaced, perVolley.get(i).get(k));
                else worstCont = Math.min(worstCont, perVolley.get(i).get(k));
            }
        }
        System.out.println(rep.substring(0, rep.indexOf("Per-ball rows")).trim());
        System.out.printf("Worst true contact speed for balls 2-4: continuous %.0f, paced %.0f (target 1407)%n",
                worstCont, worstPaced);
        check(worstPaced > worstCont, "Paced keeps later balls faster than continuous");
        System.out.println("Volley Consistency Test simulation: 12 volleys, labels, undo, LED cues and files passed");
    }
}
