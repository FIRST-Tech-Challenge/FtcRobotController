package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Single-stage transfer driven by two mechanically linked motors on Expansion Hub ports 2 and 3
 * (since 2026-11-18 transfer_motor_2 drives stage 1 with transfer_motor_1; the back rollers it used
 * to drive as a separate stage 2 were removed). Both motors always receive the same open-loop
 * stage-1 command. Motor 1's own encoder on port 2 and both motor currents are read for telemetry
 * only. There is no stall limit or safety cut-off (removed 2026-11-19 by request): stage 1 always
 * gets the controller's command, even while stalled against a jam.
 */
public class TransferSubsystem {

    public static final String HW_TRANSFER_MOTOR = "transfer_motor_1";
    public static final String HW_TRANSFER_MOTOR_2 = "transfer_motor_2";
    public static final String HW_FULL_SENSOR = "distance_sensor"; // Expansion Hub digital port 0
    // Both linked transfer motors use this direction (motor 2 changed from REVERSE on 2026-11-18 when it
    // was moved onto the stage 1 linkage). A wrong direction makes the two motors fight each other.
    public static final DcMotor.Direction MOTOR_DIRECTION = DcMotor.Direction.FORWARD;
    public static final double TRANSFER_POWER = 1.0;
    // Stage 1: goBILDA 5203 Yellow Jacket 6000 RPM (1:1), encoder counted at motor 1's shaft.
    public static final double STAGE_1_TICKS_PER_REV = BareMotorSpecifications.ENCODER_TICKS_PER_REV; // 28
    public static final double FULL_CONFIRM_TIME_SECONDS = 0.5;
    // After full confirmation stage 1 keeps running this long to seat the fourth ball.
    public static final double STAGE_1_FULL_RUN_ON_SECONDS = 0.5;
    // goBILDA 3124-0001-0001 digital mode: HIGH = object within the configured threshold.
    public static final boolean FULL_SENSOR_ACTIVE_STATE = true;

    private final DcMotorEx motor;
    private final DcMotorEx motor2;
    private final DigitalChannel fullSensor;
    private final TransferController controller = new TransferController(TRANSFER_POWER, FULL_CONFIRM_TIME_SECONDS);
    private boolean sensorState;
    private double current1 = Double.NaN, current2 = Double.NaN, lastCurrentSample = Double.NEGATIVE_INFINITY;
    private double commandedPower = 0.0;
    private boolean feeding = false;
    private boolean loadingPaused = false;
    private boolean forcedStage1Loading = false;
    private double stage1MeasuredRpm = Double.NaN;

    public TransferSubsystem(HardwareMap hardwareMap) {
        motor = initMotor(hardwareMap, HW_TRANSFER_MOTOR);
        motor2 = initMotor(hardwareMap, HW_TRANSFER_MOTOR_2);

        fullSensor = hardwareMap.get(DigitalChannel.class, HW_FULL_SENSOR);
        fullSensor.setMode(DigitalChannel.Mode.INPUT);
        sensorState = fullSensor.getState();
    }

    /** Both linked motors share one configuration: open-loop full power, motor 1's encoder is telemetry only. */
    private static DcMotorEx initMotor(HardwareMap hardwareMap, String name) {
        DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
        m.setPower(0.0);
        m.setDirection(MOTOR_DIRECTION);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return m;
    }

    /** Call once per active loop. The sensor's placement/threshold must indicate four balls loaded. */
    public void update() {
        update(false, false);
    }

    /**
     * Feed runs stage 1 (both motors) regardless of the full sensor; caller owns gate timing and
     * interlocks, including immediate continuous feed and closed-gate timed recovery in competition TeleOp.
     * Otherwise stage 1 loads unless full or paused for aiming/gate delay.
     * Full is confirmed after 0.5 s of continuous HIGH; stage 1 runs on for 0.5 s to seat the fourth
     * ball, then stays stopped while HIGH. LOW clears full immediately and resumes loading unless paused.
     * Pausing or feeding cancels the run-on.
     */
    public void update(boolean feedBalls, boolean pauseLoading) {
        update(System.nanoTime() * 1e-9, feedBalls, pauseLoading);
    }

    public void update(double now, boolean feedBalls, boolean pauseLoading) {
        update(now, feedBalls, pauseLoading, true);
    }

    /** {@code feedStage1} false holds stage 1 off during a feed; shared shot policy always passes true. */
    public void update(double now, boolean feedBalls, boolean pauseLoading, boolean feedStage1) {
        update(now, feedBalls, pauseLoading, feedStage1, false);
    }

    /** Intake-forward override loads stage 1 past full; shot/pause priorities are unchanged. */
    public void update(double now, boolean feedBalls, boolean pauseLoading, boolean feedStage1,
                       boolean forceStage1Loading) {
        readSensors(now);
        applyOutputs(now, feedBalls, pauseLoading, feedStage1, forceStage1Loading);
    }

    /** Autonomous samples before evaluating commands/conditions, then applies outputs afterward. */
    public void readSensors(double now) {
        sensorState = fullSensor.getState();
        controller.observe(now, sensorState == FULL_SENSOR_ACTIVE_STATE);
        stage1MeasuredRpm = motor.getVelocity() / STAGE_1_TICKS_PER_REV * 60.0;
        if (now-lastCurrentSample >= .2) {
            current1 = motor.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
            current2 = motor2.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
            lastCurrentSample = now;
        }
    }

    public void updateCached(double now, boolean feedBalls, boolean pauseLoading) {
        applyOutputs(now, feedBalls, pauseLoading, true);
    }

    /** Cached-sensor update with the feed's stage-1 permission and the closed-gate stage-1 run (see StaggeredFeedTiming). */
    public void updateCached(double now, boolean feedBalls, boolean pauseLoading, boolean feedStage1,
                             boolean forceStage1Loading) {
        applyOutputs(now, feedBalls, pauseLoading, feedStage1, forceStage1Loading);
    }

    private void applyOutputs(double now, boolean feedBalls, boolean pauseLoading, boolean feedStage1) {
        applyOutputs(now, feedBalls, pauseLoading, feedStage1, false);
    }

    private void applyOutputs(double now, boolean feedBalls, boolean pauseLoading, boolean feedStage1,
                              boolean forceStage1Loading) {
        controller.update(now, sensorState == FULL_SENSOR_ACTIVE_STATE, feedBalls, pauseLoading,
                feedStage1, forceStage1Loading);
        feeding = feedBalls;
        loadingPaused = pauseLoading;
        forcedStage1Loading = forceStage1Loading && !feedBalls && !pauseLoading;
        // No stall limit: stage 1 always gets the controller's command. Both motors drive the same
        // linkage, so they always share one command.
        commandedPower = controller.stage1();
        motor.setPower(commandedPower);
        motor2.setPower(commandedPower);
    }

    public double getStage1MeasuredRpm() { return stage1MeasuredRpm; }

    public boolean isFull() {
        return controller.full();
    }

    public boolean isSensorHigh() { return sensorState == FULL_SENSOR_ACTIVE_STATE; }
    public boolean isFeeding() { return feeding; }
    public boolean isLoading() { return !feeding && commandedPower > 0; }

    public void pause() {
        commandedPower = 0;
        feeding = false; loadingPaused = true; forcedStage1Loading = false; controller.pause();
        RobotShutdown.stopAll(() -> motor.setPower(0), () -> motor2.setPower(0));
    }

    public void stop() {
        commandedPower = 0.0;
        controller.stop();
        feeding = false;
        loadingPaused = false;
        forcedStage1Loading = false;
        RobotShutdown.stopAll(() -> motor.setPower(0), () -> motor2.setPower(0));
    }

    public void addTelemetry(Telemetry telemetry) {
        addTelemetry(telemetry, System.nanoTime() * 1e-9);
    }

    /** Compact match view: what the transfer is doing, without motor powers or sensor internals. */
    public void addDriverTelemetry(Telemetry telemetry) {
        telemetry.addData("Transfer", feeding ? "FEEDING SHOT"
                : isFull() ? "FULL"
                : forcedStage1Loading ? "MANUAL LOADING"
                : loadingPaused ? "PAUSED" : "LOADING");
    }

    /** Put this summary before verbose diagnostics so a stopped transfer has a visible reason. */
    public void addStatusTelemetry(Telemetry telemetry) {
        telemetry.addData("TRANSFER STATUS", "S1 %.2f (both motors) | %s | sensor %s",
                commandedPower,
                feeding ? "FEEDING SHOT"
                        : forcedStage1Loading ? "MANUAL STAGE 1 LOADING"
                        : isFull() && commandedPower > 0 ? "FULL: STAGE 1 RUN-ON (0.5 s)"
                        : isFull() ? "FULL: waiting for LOW or feed"
                        : loadingPaused ? "LOADING PAUSED BY OPMODE" : "LOADING",
                isSensorHigh() ? "HIGH" : "LOW");
    }

    public void addTelemetry(Telemetry telemetry, double now) {
        telemetry.addData("Transfer Stage 1 Devices", "%s | %s + %s | %s (linked)",
                HW_TRANSFER_MOTOR, motor.getConnectionInfo(), HW_TRANSFER_MOTOR_2, motor2.getConnectionInfo());
        telemetry.addData("Transfer current (A, 5 Hz)", "motor 1 %.2f | motor 2 %.2f",current1,current2);
        telemetry.addData("Transfer Mode", feeding ? "FEEDING SHOT"
                : forcedStage1Loading ? "MANUAL STAGE 1 LOADING"
                : isFull() && commandedPower > 0 ? "FULL: STAGE 1 RUN-ON (0.5 s)"
                : isFull() ? "FULL: HOLDING WHILE HIGH"
                : loadingPaused ? "WAITING FOR SHOT / GATE" : "AUTOMATIC LOADING");
        telemetry.addData("Transfer Full Sensor", "%s | digital %s",
                isFull() ? "FULL" : sensorState == FULL_SENSOR_ACTIVE_STATE ? "CONFIRMING FULL" : "SPACE AVAILABLE",
                sensorState ? "HIGH" : "LOW");
        telemetry.addData("Transfer Full Confirmation (s)", "%.2f / %.2f",
                controller.confirmingSeconds(now), FULL_CONFIRM_TIME_SECONDS);
        telemetry.addData("Transfer Stage 1 Power", "%.2f (both motors %s, open loop)", commandedPower, MOTOR_DIRECTION);
        telemetry.addData("Transfer Stage 1 Speed (RPM, motor encoder)", "%.0f", stage1MeasuredRpm);
    }
}
