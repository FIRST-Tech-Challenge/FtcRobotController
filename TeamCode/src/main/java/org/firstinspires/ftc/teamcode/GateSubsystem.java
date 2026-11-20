package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Gate wired directly to Expansion Hub servo port 3, without the 6 V injector. */
public class GateSubsystem {

    public static final String HW_GATE_SERVO = "gate_servo";
    public static final String SERVO_MODEL = "goBILDA Dual Mode Super Speed (2000-0025-0004)";
    // Servo must be programmed in positional mode: 300 deg over 500..2500 us.
    // https://www.gobilda.com/2000-series-dual-mode-servo-25-4-super-speed/
    public static final int MIN_PULSE_US = 500;
    public static final int MAX_PULSE_US = 2500;
    public static final double SERVO_TRAVEL_DEG = 300.0;
    public static final Servo.Direction SERVO_DIRECTION = Servo.Direction.REVERSE;
    public static final double CLOSED_ANGLE_DEG = SERVO_TRAVEL_DEG / 2.0;
    public static final double GATE_OPEN_TRAVEL_DEG = 90.0;
    // REVERSE maps logical 0.5 -> 0.2 to physical 0.5 -> 0.8: 1500 -> 2100 us, 90 deg CW.
    public static final double OPEN_ANGLE_DEG = CLOSED_ANGLE_DEG - GATE_OPEN_TRAVEL_DEG;
    public static final double TRIGGER_DEADBAND = 0.1;
    // Catalog reference only; direct-hub loaded speed has not been measured.
    public static final double SECONDS_PER_60_DEG_AT_6V = 0.043;
    public static final double RATED_OPEN_TRAVEL_SECONDS = GATE_OPEN_TRAVEL_DEG
            / 60.0 * SECONDS_PER_60_DEG_AT_6V;
    // The gate receives its target immediately; start the transfer feed 0.10 s after that command.
    // This is the operator-requested feed delay, not an additional servo settling allowance.
    public static final double FEED_DELAY_AFTER_OPEN_COMMAND_SECONDS = 0.10;

    private final Servo servo;
    private boolean open = false;
    private final DoubleSupplier clock;
    private final double feedDelaySeconds;
    private double openedAt;

    public GateSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, () -> System.nanoTime() * 1e-9);
    }

    public GateSubsystem(HardwareMap hardwareMap, DoubleSupplier clock) {
        this(hardwareMap, clock, FEED_DELAY_AFTER_OPEN_COMMAND_SECONDS);
    }

    /** Per-OpMode feed delay override; other callers keep the shared default. */
    public GateSubsystem(HardwareMap hardwareMap, double feedDelaySeconds) {
        this(hardwareMap, () -> System.nanoTime() * 1e-9, feedDelaySeconds);
    }

    public GateSubsystem(HardwareMap hardwareMap, DoubleSupplier clock, double feedDelaySeconds) {
        if (!Double.isFinite(feedDelaySeconds) || feedDelaySeconds < 0)
            throw new IllegalArgumentException("Invalid gate feed delay");
        this.clock = clock;
        this.feedDelaySeconds = feedDelaySeconds;
        servo = hardwareMap.get(Servo.class, HW_GATE_SERVO);
        if (!(servo instanceof PwmControl)) {
            throw new IllegalArgumentException(HW_GATE_SERVO + " must support PwmControl to configure 500-2500 us PWM");
        }
        servo.setDirection(SERVO_DIRECTION);
        ((PwmControl) servo).setPwmRange(new PwmControl.PwmRange(MIN_PULSE_US, MAX_PULSE_US));
        close();
    }

    public void setOpen(boolean open) {
        boolean opening = open && !this.open;
        this.open = open;
        double angleDeg = open ? OPEN_ANGLE_DEG : CLOSED_ANGLE_DEG;
        // Immediate full target for maximum available servo speed; no software ramp.
        servo.setPosition(angleDeg / SERVO_TRAVEL_DEG);
        if (opening) openedAt = clock.getAsDouble();
    }

    public void close() {
        setOpen(false);
    }

    /** Gate is commanded open and its command-to-feed delay has elapsed; no position feedback. */
    public boolean isOpenForFeeding() {
        return open && clock.getAsDouble() - openedAt >= feedDelaySeconds;
    }

    public boolean isOpenCommanded() { return open; }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Gate Servo", SERVO_MODEL);
        telemetry.addData("Gate Power", "Direct Expansion Hub port 3 (no 6 V injector)");
        telemetry.addData("Gate PWM Range (us)", "%d - %d | %s",
                MIN_PULSE_US, MAX_PULSE_US, SERVO_DIRECTION);
        telemetry.addData("Gate Open Travel (deg)", GATE_OPEN_TRAVEL_DEG);
        telemetry.addData("Gate Command-to-Feed Delay (s)", feedDelaySeconds);
        telemetry.addData("Gate Feed Allowed", isOpenForFeeding());
        telemetry.addData("Gate Target (deg)", "%.0f (%s)",
                open ? OPEN_ANGLE_DEG : CLOSED_ANGLE_DEG, open ? "OPEN" : "CLOSED");
    }
}
