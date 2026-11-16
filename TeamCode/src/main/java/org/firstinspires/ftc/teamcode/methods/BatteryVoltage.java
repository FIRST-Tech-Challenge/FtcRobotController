package org.firstinspires.ftc.teamcode.methods;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.function.DoubleSupplier;

/**
 * Shared, rate-limited battery voltage.
 *
 * Every VoltageSensor.getVoltage() call is a separate hub command (~2-3 ms) and is NOT
 * covered by bulk caching. Launcher, Transfer and Intake used to read every hub's sensor
 * on every loop, costing up to ~8 commands per loop. Create ONE instance per OpMode and
 * pass it to all three; voltage is then read at most every MIN_READ_INTERVAL_SECONDS.
 */
public final class BatteryVoltage {
    /** Battery voltage changes slowly compared with a control loop; 10 Hz is plenty. */
    public static double MIN_READ_INTERVAL_SECONDS = 0.10;

    private final Iterable<VoltageSensor> sensors;
    private final DoubleSupplier clock;
    private double lastReadTime = Double.NaN;
    private double voltage = Double.NaN;
    private int reads;

    public BatteryVoltage(HardwareMap hardwareMap) {
        this(hardwareMap.voltageSensor);
    }

    public BatteryVoltage(Iterable<VoltageSensor> sensors) {
        this(sensors, () -> System.nanoTime() / 1e9);
    }

    BatteryVoltage(Iterable<VoltageSensor> sensors, DoubleSupplier clock) {
        this.sensors = sensors;
        this.clock = clock;
    }

    /**
     * Lowest valid hub voltage, refreshed when the interval has elapsed.
     * Returns NaN until a valid reading exists; a failed refresh keeps the last valid value.
     */
    public double get() {
        double now = clock.getAsDouble();
        if (Double.isNaN(lastReadTime) || !(now - lastReadTime < MIN_READ_INTERVAL_SECONDS)
                || now < lastReadTime) {
            lastReadTime = now;
            double fresh = readNow();
            if (Double.isFinite(fresh)) voltage = fresh;
        }
        return voltage;
    }

    /** Number of hardware refreshes performed; used by tests and loop diagnostics. */
    public int readCount() { return reads; }

    private double readNow() {
        if (sensors == null) return Double.NaN;
        reads++;
        double minimum = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : sensors) {
            double v = sensor.getVoltage();
            if (Double.isFinite(v) && v > 0) minimum = Math.min(minimum, v);
        }
        return Double.isFinite(minimum) ? minimum : Double.NaN;
    }
}
