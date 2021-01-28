package teamcode.common;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * Provides an easy way to debug.
 */
public class Debug {

    /**
     * How many lines to display at max in the telemetry window.
     */
    private static final int MAX_NUM_LINES = 10;

    private static final Telemetry telemetry;
    private static final List<Object> lines;

    static {
        if (AbstractOpMode.currentOpMode() == null) {
            throw new IllegalStateException("Cannot use Debug unless AbstractOpMode is initialized.");
        }
        telemetry = AbstractOpMode.currentOpMode().telemetry;
        lines = new ArrayList<>();
    }

    private Debug() {
        // Brian why would you instantiate Debug?
    }

    public static synchronized void log(Object message) {
        lines.add(message);
        if (lines.size() > MAX_NUM_LINES) {
            lines.remove(0);
        }
        updateTelemetry();
    }

    public static void clear() {
        lines.clear();
        updateTelemetry();
    }

    private static void updateTelemetry() {
        for (Object line : lines) {
            telemetry.addData("Debug", line);
        }
        telemetry.update();
    }

}
