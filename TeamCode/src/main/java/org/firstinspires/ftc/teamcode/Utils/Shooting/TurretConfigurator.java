package org.firstinspires.ftc.teamcode.Utils.Shooting;

import org.firstinspires.ftc.teamcode.Utils.Controls.ControlsTrigger;
import org.firstinspires.ftc.teamcode.Utils.Controls.GamepadButton;
import org.firstinspires.ftc.teamcode.Utils.LinearOpMode2026;
import org.firstinspires.ftc.teamcode.Utils.MiscUtils;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

/**
 * Sets flywheel RPM and hood angle from a distance -> (RPM, hood) curve.
 * <p>
 * The curve starts as a line through the two seed points. Whenever you adjust the
 * flywheel or hood with the dpad, the current distance + values are saved as a new
 * point, so the curve keeps getting better as you tune. Points are saved to a file
 * and loaded again next run.
 * <p>
 * Controls (gamepad1):
 *   Circle            : switch between Flywheel / Hood tuning
 *   Dpad Up / Down    : raise / lower the selected value (adds a point at this distance)
 *   Dpad Right / Left : change the increment size
 *   Square            : reset the curve back to the two seed points
 */
public class TurretConfigurator {

    static final int MAX_RPM = 6000;
    // goBILDA 6000 RPM (1:1) motor encoder
    static final double TICKS_PER_REV = 28;
    static final double HOOD_MIN = 0.05;
    static final double HOOD_MAX = 0.6;

    // A new point within this many inches of an existing point replaces it
    static final double MERGE_DISTANCE = 2.0;

    static final String CURVE_PATH = MiscUtils.dataFolder + "/shooterCurve.txt";

    // Seed points: {distanceToGoal, flywheel RPM, hood}
    static final double[][] SEED_POINTS = {
            {40.23, 2200, 0.20},
            {52.65, 2400, 0.24},
    };

    /** distance -> {rpm, hood}, sorted by distance */
    private final TreeMap<Double, double[]> curve = new TreeMap<>();

    private final LinearOpMode2026 op;

    private final ControlsTrigger dpadUpTrigger;
    private final ControlsTrigger dpadDownTrigger;
    private final ControlsTrigger dpadLeftTrigger;
    private final ControlsTrigger dpadRightTrigger;
    private final ControlsTrigger circleTrigger;
    private final ControlsTrigger squareTrigger;

    int velocity = 0;
    double hoodAngle = 0;

    int[] velocityIncrements = {10, 100, 500};
    double[] servoIncrements = {0.01, 0.05, 0.1};
    int selectedTune = 0;
    int selectedIncrement = 0;

    public TurretConfigurator(LinearOpMode2026 op) {
        this.op = op;

        dpadUpTrigger = new ControlsTrigger(op.gamepad1, GamepadButton.GButton.dpad_up);
        dpadDownTrigger = new ControlsTrigger(op.gamepad1, GamepadButton.GButton.dpad_down);
        dpadLeftTrigger = new ControlsTrigger(op.gamepad1, GamepadButton.GButton.dpad_left);
        dpadRightTrigger = new ControlsTrigger(op.gamepad1, GamepadButton.GButton.dpad_right);
        circleTrigger = new ControlsTrigger(op.gamepad1, GamepadButton.GButton.circle);
        squareTrigger = new ControlsTrigger(op.gamepad1, GamepadButton.GButton.square);

        if (!loadCurve()) {
            resetCurve();
        }
    }

    /**
     * Call every loop.
     * @param distance current distance to the goal (same units as the seed points)
     */
    public void runTurret(double distance) {
        // Each trigger is true only once per press
        boolean dpadUp = dpadUpTrigger.IsTriggered();
        boolean dpadDown = dpadDownTrigger.IsTriggered();
        boolean dpadLeft = dpadLeftTrigger.IsTriggered();
        boolean dpadRight = dpadRightTrigger.IsTriggered();

        if (squareTrigger.IsTriggered()) {
            resetCurve();
            saveCurve();
        }

        //Changes what is tuned
        if (circleTrigger.IsTriggered()) {
            selectedTune += 1;
            if (selectedTune > 1)
                selectedTune = 0;
        }

        //Changes tuning increment
        if (dpadRight) {
            selectedIncrement += 1;
            if (selectedIncrement > 2)
                selectedIncrement = 0;
        }
        if (dpadLeft) {
            selectedIncrement -= 1;
            if (selectedIncrement < 0)
                selectedIncrement = 2;
        }

        // Start from what the curve says for this distance
        double[] values = getValues(distance);
        velocity = (int) Math.round(values[0]);
        hoodAngle = values[1];

        //Tuning Math
        boolean changed = false;
        if (selectedTune == 0) { //Flywheel
            if (dpadUp) {
                velocity = Math.min(velocity + velocityIncrements[selectedIncrement], MAX_RPM);
                changed = true;
            }
            if (dpadDown) {
                velocity = Math.max(velocity - velocityIncrements[selectedIncrement], 0);
                changed = true;
            }

            op.telemetry.addLine("Flywheel Tuning");
            op.telemetry.addData("Increment Value ", velocityIncrements[selectedIncrement]);

        } else { // Hood
            if (dpadUp) {
                hoodAngle = Math.min(hoodAngle + servoIncrements[selectedIncrement], HOOD_MAX);
                changed = true;
            }
            if (dpadDown) {
                hoodAngle = Math.max(hoodAngle - servoIncrements[selectedIncrement], HOOD_MIN);
                changed = true;
            }
            op.telemetry.addLine("Hood Tuning");
            op.telemetry.addData("Increment Value ", servoIncrements[selectedIncrement]);
        }

        // A change at this distance becomes a point on the curve
        if (changed) {
            addPoint(distance, velocity, hoodAngle);
            saveCurve();
        }

        // velocity is in RPM; setVelocity takes ticks per second
        op.flywheel.setVelocity(velocity * TICKS_PER_REV / 60.0);
        op.hood.setPosition(hoodAngle);

        showTelemetry(distance);
    }

    /** @return {rpm, hood} for this distance, clamped to safe ranges */
    public double[] getValues(double distance) {
        double rpm = interpolate(distance, 0);
        double hood = interpolate(distance, 1);
        return new double[]{
                MiscUtils.Clamp(rpm, 0, MAX_RPM),
                MiscUtils.Clamp(hood, HOOD_MIN, HOOD_MAX)
        };
    }

    /**
     * Straight lines between neighboring points. Outside the tuned range it
     * keeps following the line through the two nearest end points.
     */
    private double interpolate(double distance, int index) {
        if (curve.size() == 1) return curve.firstEntry().getValue()[index];

        Map.Entry<Double, double[]> lower = curve.floorEntry(distance);
        Map.Entry<Double, double[]> upper = curve.higherEntry(distance);

        if (lower == null) { // Before the first point
            lower = curve.firstEntry();
            upper = curve.higherEntry(lower.getKey());
        } else if (upper == null) { // After the last point
            upper = curve.lastEntry();
            lower = curve.lowerEntry(upper.getKey());
        }

        double x0 = lower.getKey(), x1 = upper.getKey();
        double y0 = lower.getValue()[index], y1 = upper.getValue()[index];
        return y0 + (distance - x0) * (y1 - y0) / (x1 - x0);
    }

    private void addPoint(double distance, double rpm, double hood) {
        // Replace any point that is basically at the same distance
        curve.subMap(distance - MERGE_DISTANCE, true, distance + MERGE_DISTANCE, true).clear();
        curve.put(distance, new double[]{rpm, hood});
    }

    private void resetCurve() {
        curve.clear();
        for (double[] p : SEED_POINTS) {
            curve.put(p[0], new double[]{p[1], p[2]});
        }
    }

    /** File format: one point per line, "distance,rpm,hood" */
    private void saveCurve() {
        try (FileWriter writer = new FileWriter(CURVE_PATH)) {
            for (Map.Entry<Double, double[]> e : curve.entrySet()) {
                writer.write(e.getKey() + "," + e.getValue()[0] + "," + e.getValue()[1] + "\n");
            }
        } catch (Exception e) {
            op.telemetry.addData("Curve save error", e.toString());
        }
    }

    /** @return true if at least two points were loaded */
    private boolean loadCurve() {
        if (!MiscUtils.checkFile(CURVE_PATH)) return false;

        List<double[]> points = new ArrayList<>();
        try (BufferedReader reader = new BufferedReader(new FileReader(CURVE_PATH))) {
            String line;
            while ((line = reader.readLine()) != null) {
                String[] parts = line.trim().split(",");
                if (parts.length != 3) continue;
                points.add(new double[]{
                        Double.parseDouble(parts[0]),
                        Double.parseDouble(parts[1]),
                        Double.parseDouble(parts[2])
                });
            }
        } catch (Exception e) {
            op.telemetry.addData("Curve load error", e.toString());
            return false;
        }

        if (points.size() < 2) return false;

        curve.clear();
        for (double[] p : points) {
            curve.put(p[0], new double[]{p[1], p[2]});
        }
        return true;
    }

    void showTelemetry(double distance) {
        op.telemetry.addData("Distance To Goal", "%.2f", distance);
        op.telemetry.addData("Flywheel Wanted RPM ", velocity);
        op.telemetry.addData("Flywheel Current RPM ", "%.0f", op.flywheel.getVelocity() * 60.0 / TICKS_PER_REV);
        op.telemetry.addData("Hood Angle", "%.2f", hoodAngle);

        op.telemetry.addLine("Curve points (distance, RPM, hood):");
        for (Map.Entry<Double, double[]> e : curve.entrySet()) {
            op.telemetry.addLine(String.format("  %.2f, %.0f, %.2f", e.getKey(), e.getValue()[0], e.getValue()[1]));
        }
    }

}
