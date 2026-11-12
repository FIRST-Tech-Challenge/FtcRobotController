package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Three independent goBILDA 3118-0808-0002 PWM indicators on Control Hub servo ports 3 (led_intake),
 * 4 (led_shooter) and 5 (led_status); missing lights are reported, optional hardware.
 */
public final class RobotStatusLights {
    public static final String HW_INTAKE = "led_intake", HW_SHOOTER = "led_shooter", HW_STATUS = "led_status";
    public static final double PWM_MIN_US = 600, PWM_MAX_US = 2400;
    public static final double BLINK_HALF_PERIOD_SECONDS = .25;
    public enum Color {
        OFF(600), RED(1100), ORANGE(1200), YELLOW(1300), GREEN(1500), BLUE(1700), VIOLET(1900), WHITE(2400);
        public final double position;
        Color(double pulseUs) { position = (pulseUs - PWM_MIN_US) / (PWM_MAX_US - PWM_MIN_US); }
    }
    private final Servo[] lights = new Servo[3];
    private final Color[] colors = new Color[3];
    private final String missing;

    public RobotStatusLights(HardwareMap hardwareMap) {
        String[] names = {HW_INTAKE, HW_SHOOTER, HW_STATUS};
        StringBuilder absent = new StringBuilder();
        for (int i = 0; i < lights.length; i++) {
            lights[i] = hardwareMap.tryGet(Servo.class, names[i]);
            if (lights[i] == null) { if (absent.length() > 0) absent.append(", "); absent.append(names[i]); continue; }
            lights[i].setDirection(Servo.Direction.FORWARD);
            lights[i].scaleRange(0, 1);
            if (lights[i] instanceof PwmControl)
                ((PwmControl) lights[i]).setPwmRange(new PwmControl.PwmRange(PWM_MIN_US, PWM_MAX_US));
        }
        missing = absent.toString();
        stop();
    }
    public void set(int index, Color color) {
        if (colors[index] == color) return;
        colors[index] = color;
        if (lights[index] != null) lights[index].setPosition(color.position);
    }
    public void showIntake(double now, boolean full, boolean forward, boolean reverse,
                           double power, boolean clearing) {
        set(0, intakeColor(now, full, forward, reverse, power, clearing));
    }
    public static Color intakeColor(double now, boolean full, boolean forward, boolean reverse,
                                    double power, boolean clearing) {
        if (clearing) return blink(now, Color.ORANGE);
        if (forward && reverse) return Color.OFF;
        if (reverse) return Color.ORANGE;
        if (forward) return Color.BLUE;
        if (full) return Color.GREEN;
        return power == 0 ? Color.OFF : power > 0 ? Color.BLUE : Color.ORANGE;
    }
    public void showShooter(double now, FiringCoordinator.State state, boolean manualRequest,
                            boolean ready, boolean spinning) {
        set(1, shooterColor(now, state, manualRequest, ready, spinning));
    }
    public static Color shooterColor(double now, FiringCoordinator.State state, boolean manualRequest,
                                     boolean ready, boolean spinning) {
        if (state == FiringCoordinator.State.INHIBITED || state == FiringCoordinator.State.FAULT) return Color.RED;
        if (state == FiringCoordinator.State.FEEDING || state == FiringCoordinator.State.GATE_OPENING)
            return manualRequest ? blink(now, Color.VIOLET) : Color.VIOLET;
        if (state == FiringCoordinator.State.RECOVERY) return Color.YELLOW;
        if (ready) return Color.GREEN;
        return spinning || state == FiringCoordinator.State.PREPARING ? Color.YELLOW : Color.OFF;
    }
    public void showReference(boolean hardwareAvailable, boolean homed, boolean freshFieldPose) {
        set(2, !hardwareAvailable ? Color.RED : !homed ? Color.YELLOW : freshFieldPose ? Color.GREEN : Color.ORANGE);
    }
    private static Color blink(double now, Color color) {
        return ((long) Math.floor(now / BLINK_HALF_PERIOD_SECONDS) & 1) == 0 ? color : Color.OFF;
    }
    public void stop() { for (int i = 0; i < lights.length; i++) set(i, Color.OFF); }
    /** Compact match view: only reports lights that are missing from the configuration. */
    public void addWarningTelemetry(Telemetry telemetry) {
        if (!missing.isEmpty()) telemetry.addData("Missing status lights", missing);
    }
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Lights intake / shooter / reference", "%s / %s / %s", colors[0], colors[1], colors[2]);
        telemetry.addData("Missing status lights", missing.isEmpty() ? "none" : missing);
    }
}
