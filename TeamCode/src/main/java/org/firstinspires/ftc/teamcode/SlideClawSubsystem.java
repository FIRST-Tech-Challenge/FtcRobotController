package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Claw end effector on the linear slide: goBILDA Super Speed servo on Control Hub servo port 0 (CH servo 3-5 are the status lights).
 * Picks up nectar/pollen and drops it into flowers. With the shot-blocker end effector mounted the
 * servo is unplugged, so commands simply have no effect.
 */
public class SlideClawSubsystem {

    public static final String HW_CLAW_SERVO = "claw_servo";
    public static final String SERVO_MODEL = "goBILDA Dual Mode Super Speed (2000-0025-0004)";
    // Positional mode: 300 deg over 500..2500 us, same as the gate servo.
    public static final int MIN_PULSE_US = 500;
    public static final int MAX_PULSE_US = 2500;
    public static final Servo.Direction SERVO_DIRECTION = Servo.Direction.FORWARD;
    // PLACEHOLDERS: find the real values with GP1 D-pad in "Test: Linear Slide + Claw".
    public static final double OPEN_POSITION = 0.40;
    public static final double CLOSED_POSITION = 0.60;
    public static final boolean POSITIONS_MEASURED = false;

    private final Servo servo;
    private boolean open = false;
    private double commandedPosition;

    public SlideClawSubsystem(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, HW_CLAW_SERVO);
        servo.setDirection(SERVO_DIRECTION);
        if (servo instanceof PwmControl) {
            ((PwmControl) servo).setPwmRange(new PwmControl.PwmRange(MIN_PULSE_US, MAX_PULSE_US));
        }
        close();
    }

    public void open() { open = true; setPosition(OPEN_POSITION); }
    public void close() { open = false; setPosition(CLOSED_POSITION); }
    public void toggle() { if (open) close(); else open(); }

    /** Test only: move to any position to measure open/closed values. */
    public void setPositionForTest(double position) { setPosition(position); }

    private void setPosition(double position) {
        commandedPosition = Math.max(0.0, Math.min(1.0, position));
        servo.setPosition(commandedPosition);
    }

    public boolean isOpenCommanded() { return open; }
    public double getCommandedPosition() { return commandedPosition; }

    public void addDriverTelemetry(Telemetry telemetry) {
        telemetry.addData("Claw (GP1 B)", "%s%s", open ? "OPEN" : "CLOSED",
                POSITIONS_MEASURED ? "" : " | POSITIONS NOT MEASURED");
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Claw Servo", "%s | %d - %d us | %s", SERVO_MODEL, MIN_PULSE_US, MAX_PULSE_US, SERVO_DIRECTION);
        telemetry.addData("Claw Position (0-1)", "%.3f (open %.3f / closed %.3f)",
                commandedPosition, OPEN_POSITION, CLOSED_POSITION);
    }
}
