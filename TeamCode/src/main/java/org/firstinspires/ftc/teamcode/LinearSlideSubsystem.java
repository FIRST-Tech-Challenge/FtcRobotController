package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Side linear slide: one Axon MAX+ in continuous mode on Expansion Hub servo port 5 (6 V injector),
 * with its analog feedback on Expansion Hub analog port 0. Toggles between fully down and fully up.
 * The end effector (claw or shot blocker) is separate; see {@link SlideClawSubsystem}.
 *
 * <p>The analog angle only covers one turn, so turns are counted from INIT. The slide must be
 * fully down when the OpMode is initialized.
 */
public class LinearSlideSubsystem {

    public static final String HW_SLIDE_SERVO = "slide_servo";
    public static final String HW_SLIDE_ENCODER = "slide_encoder";
    // Set both so that positive power raises the slide and the reported angle increases going up.
    public static final DcMotorSimple.Direction SERVO_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final boolean ENCODER_INVERTED = false;
    // Full CR power maps to the Axon's programmed pulse limits.
    public static final int MIN_PULSE_US = AxonMaxPlus.MIN_PULSE_US;
    public static final int MAX_PULSE_US = AxonMaxPlus.MAX_PULSE_US;

    private final CRServo servo;
    private final AnalogInput encoder;
    private final DoubleSupplier clock;
    private final LinearSlideController controller = new LinearSlideController(ENCODER_INVERTED);
    private double voltage, rawDeg;

    public LinearSlideSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, () -> System.nanoTime() * 1e-9);
    }

    public LinearSlideSubsystem(HardwareMap hardwareMap, DoubleSupplier clock) {
        this.clock = clock;
        servo = hardwareMap.get(CRServo.class, HW_SLIDE_SERVO);
        encoder = hardwareMap.get(AnalogInput.class, HW_SLIDE_ENCODER);
        servo.setDirection(SERVO_DIRECTION);
        if (servo instanceof PwmControl) {
            ((PwmControl) servo).setPwmRange(new PwmControl.PwmRange(MIN_PULSE_US, MAX_PULSE_US));
        }
        servo.setPower(0.0);
        readEncoder();
        controller.zero(rawDeg);
    }

    private void readEncoder() {
        voltage = encoder.getVoltage();
        double max = encoder.getMaxVoltage();
        double fraction = max > 0 ? voltage / max : 0.0;
        rawDeg = Math.max(0.0, Math.min(1.0, fraction)) * 360.0;
    }

    /** Call once per loop, after {@code RobotLoop.begin()}. Reads the encoder and writes servo power. */
    public void update() {
        readEncoder();
        servo.setPower(controller.update(clock.getAsDouble(), rawDeg));
    }

    /** INIT loop: keep counting turns but leave the slide unpowered. */
    public void trackWithoutDriving() {
        readEncoder();
        controller.observe(rawDeg);
        servo.setPower(0.0);
    }

    public void toggle() { controller.toggle(clock.getAsDouble()); }
    public void raise() { controller.raise(clock.getAsDouble()); }
    public void lower() { controller.lower(clock.getAsDouble()); }

    /** Test only: open-loop power with no limits; NaN = released (unpowered until the next toggle). */
    public void setManualPower(double power) { controller.setManualPower(power, clock.getAsDouble()); }

    /** Test only: declare the current position fully down. */
    public void rezero() {
        readEncoder();
        controller.zero(rawDeg);
        controller.lower(clock.getAsDouble());
    }

    public void stop() {
        controller.stop();
        servo.setPower(0.0);
    }

    public boolean isUpCommanded() { return controller.isUpCommanded(); }
    public double getPositionDeg() { return controller.getPositionDeg(); }
    public LinearSlideController.State getState() { return controller.getState(); }

    public void addDriverTelemetry(Telemetry telemetry) {
        LinearSlideController.State state = controller.getState();
        if (state == LinearSlideController.State.FAULT) {
            // The button toggles, so the next press reverses the faulted move rather than repeating it.
            telemetry.addData("Slide (GP2 LS)", "FAULT: %s | next press moves %s", controller.getFaultReason(),
                    controller.isUpCommanded() ? "DOWN" : "UP");
        } else {
            telemetry.addData("Slide (GP2 LS)", "%s | %.0f / %.0f deg%s", state,
                    controller.getPositionDeg(), controller.targetDeg(),
                    LinearSlideController.TRAVEL_MEASURED ? "" : " | TRAVEL NOT MEASURED");
        }
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Slide State", "%s%s", controller.getState(),
                controller.getFaultReason().isEmpty() ? "" : ": " + controller.getFaultReason());
        telemetry.addData("Slide Position / Target (deg)", "%.1f / %.1f",
                controller.getPositionDeg(), controller.targetDeg());
        telemetry.addData("Slide Turns (rev)", "%.2f", controller.getPositionDeg() / 360.0);
        telemetry.addData("Slide Encoder (V, raw deg)", "%.3f V | %.1f deg", voltage, rawDeg);
        telemetry.addData("Slide Power", "%+.2f", controller.getPower());
        telemetry.addData("Slide Max Travel (deg)", "%.1f (%s)", controller.getMaxTravelDeg(),
                LinearSlideController.TRAVEL_MEASURED ? "measured" : "PLACEHOLDER");
    }
}
