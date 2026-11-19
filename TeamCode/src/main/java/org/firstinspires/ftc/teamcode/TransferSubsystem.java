package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Two-stage transfer on Expansion Hub motor ports 2 and 3. Both stages run open-loop full power;
 * stage 1's own motor encoder on port 2 is read for speed telemetry and, with its current, for
 * {@link TransferStallGuard} thermal protection, which cuts stage 1 during a sustained stall while
 * the intake is running forward (see {@link #setIntakeRunning(boolean)}).
 */
public class TransferSubsystem {

    public static final String HW_TRANSFER_MOTOR = "transfer_motor_1";
    public static final String HW_TRANSFER_MOTOR_2 = "transfer_motor_2";
    public static final String HW_FULL_SENSOR = "distance_sensor"; // Expansion Hub digital port 0
    // Both transfer motors are wired reversed relative to the original build (corrected 2026-11-18).
    public static final DcMotor.Direction MOTOR_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction STAGE_2_MOTOR_DIRECTION = DcMotor.Direction.REVERSE;
    public static final double TRANSFER_POWER = 1.0;
    // Stage 1: goBILDA 5203 Yellow Jacket 6000 RPM (1:1), encoder counted at the motor shaft.
    public static final double STAGE_1_TICKS_PER_REV = BareMotorSpecifications.ENCODER_TICKS_PER_REV; // 28
    public static final double FULL_CONFIRM_TIME_SECONDS = 0.5;
    // After full confirmation stage 1 keeps running this long to seat the fourth ball.
    public static final double STAGE_1_FULL_RUN_ON_SECONDS = 0.5;
    // goBILDA 3124-0001-0001 digital mode: HIGH = object within the configured threshold.
    public static final boolean FULL_SENSOR_ACTIVE_STATE = true;

    private final DcMotorEx motor;
    private final DcMotorEx stage2Motor;
    private final DigitalChannel fullSensor;
    private final TransferController controller = new TransferController(TRANSFER_POWER, FULL_CONFIRM_TIME_SECONDS);
    private final TransferStallGuard stallGuard = new TransferStallGuard();
    private boolean sensorState;
    private double current1 = Double.NaN, current2 = Double.NaN, lastCurrentSample = Double.NEGATIVE_INFINITY;
    private double commandedPower = 0.0;
    private double stage2Power = 0.0;
    private boolean feeding = false;
    private boolean loadingPaused = false;
    private boolean forcedStage1Loading = false;
    private boolean intakeRunning = false;
    private double stage1MeasuredRpm = Double.NaN;

    public TransferSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, HW_TRANSFER_MOTOR);
        motor.setPower(0.0);
        motor.setDirection(MOTOR_DIRECTION);
        // Always open-loop full power; the port 2 encoder is only read for speed telemetry.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stage2Motor = hardwareMap.get(DcMotorEx.class, HW_TRANSFER_MOTOR_2);
        stage2Motor.setPower(0.0);
        stage2Motor.setDirection(STAGE_2_MOTOR_DIRECTION);
        stage2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stage2Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fullSensor = hardwareMap.get(DigitalChannel.class, HW_FULL_SENSOR);
        fullSensor.setMode(DigitalChannel.Mode.INPUT);
        sensorState = fullSensor.getState();
    }

    /** Call once per active loop. The sensor's placement/threshold must indicate four balls loaded. */
    public void update() {
        update(false, false);
    }

    /**
     * Feed runs both stages regardless of the full sensor; caller owns gate timing and interlocks,
     * including immediate continuous feed and closed-gate timed recovery in competition TeleOp.
     * Otherwise stage 1 loads unless full or paused for aiming/gate delay; stage 2 stays stopped.
     * Full is confirmed after 0.5 s of continuous HIGH; stage 1 runs on for 0.5 s to seat the fourth
     * ball, then stays stopped while HIGH. LOW clears full immediately and resumes loading unless paused.
     * Pausing or feeding cancels the run-on; stage 2 stays off outside feeding.
     */
    public void update(boolean feedBalls, boolean pauseLoading) {
        update(System.nanoTime() * 1e-9, feedBalls, pauseLoading);
    }

    public void update(double now, boolean feedBalls, boolean pauseLoading) {
        update(now, feedBalls, pauseLoading, true);
    }

    /** Optional shot sequencing: stage 2 feeds first, then stage 1 advances the next ball. */
    public void update(double now, boolean feedBalls, boolean pauseLoading, boolean feedStage1) {
        update(now, feedBalls, pauseLoading, feedStage1, false);
    }

    /** Intake-forward override loads stage 1 past full; stage 2 and shot/pause priorities are unchanged. */
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
            current2 = stage2Motor.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
            lastCurrentSample = now;
        }
    }

    public void updateCached(double now, boolean feedBalls, boolean pauseLoading) {
        applyOutputs(now, feedBalls, pauseLoading, true);
    }

    /** Cached-sensor update with staggered stage 1 and the closed-gate stage-1 run (see StaggeredFeedTiming). */
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
        if (feedBalls || !intakeRunning) {
            // Shot feeds always get the controller's full power on both stages; the stall guard never
            // cuts a shot, and each feed clears any loading lockout so stage 1 starts fresh. The guard is
            // also inactive (and cleared) whenever the intake is not running forward.
            stallGuard.reset();
            commandedPower = controller.stage1();
        } else {
            // While intaking, stage 1 pushes balls into a stopped stage 2 or closed gate, which is where
            // it can stall and overheat.
            commandedPower = stallGuard.filter(now, controller.stage1(), stage1MeasuredRpm, current1);
        }
        motor.setPower(commandedPower);
        stage2Power = controller.stage2();
        stage2Motor.setPower(stage2Power);
    }

    public double getStage1MeasuredRpm() { return stage1MeasuredRpm; }

    public boolean isFull() {
        return controller.full();
    }

    public boolean isSensorHigh() { return sensorState == FULL_SENSOR_ACTIVE_STATE; }
    public boolean isFeeding() { return feeding; }
    public boolean isLoading() { return !feeding && commandedPower > 0; }

    /**
     * The stage 1 stall guard is active only while the intake is running forward (intaking). Call once
     * per loop after setting the intake power; it applies from the next transfer update. OpModes
     * without an intake never call it, so the guard stays inactive there.
     */
    public void setIntakeRunning(boolean intakingForward) { intakeRunning = intakingForward; }
    public boolean isIntakeRunning() { return intakeRunning; }

    public boolean isStage1StallProtected() { return stallGuard.cuttingPower(); }
    public TransferStallGuard.State getStallState() { return stallGuard.state(); }

    public void pause() {
        commandedPower = stage2Power = 0; stallGuard.reset();
        feeding = false; loadingPaused = true; forcedStage1Loading = false; controller.pause();
        RobotShutdown.stopAll(() -> motor.setPower(0), () -> stage2Motor.setPower(0));
    }

    public void stop() {
        commandedPower = 0.0; stallGuard.reset();
        controller.stop();
        feeding = false;
        loadingPaused = false;
        forcedStage1Loading = false;
        stage2Power = 0.0;
        RobotShutdown.stopAll(() -> motor.setPower(0), () -> stage2Motor.setPower(0));
    }

    public void addTelemetry(Telemetry telemetry) {
        addTelemetry(telemetry, System.nanoTime() * 1e-9);
    }

    /** Compact match view: what the transfer is doing, without motor powers or sensor internals. */
    public void addDriverTelemetry(Telemetry telemetry) {
        telemetry.addData("Transfer", stallText() != null ? stallText()
                : feeding ? "FEEDING SHOT"
                : isFull() ? "FULL"
                : forcedStage1Loading ? "MANUAL LOADING"
                : loadingPaused ? "PAUSED" : "LOADING");
    }

    /** Put this summary before verbose diagnostics so a stopped transfer has a visible reason. */
    public void addStatusTelemetry(Telemetry telemetry) {
        telemetry.addData("TRANSFER STATUS", "S1 %.2f | S2 %.2f | %s | sensor %s",
                commandedPower, stage2Power,
                stallText() != null ? stallText()
                        : feeding ? "FEEDING SHOT"
                        : forcedStage1Loading ? "MANUAL STAGE 1 LOADING"
                        : isFull() && commandedPower > 0 ? "FULL: STAGE 1 RUN-ON (0.5 s)"
                        : isFull() ? "FULL: waiting for LOW or feed"
                        : loadingPaused ? "LOADING PAUSED BY OPMODE" : "LOADING",
                isSensorHigh() ? "HIGH" : "LOW");
    }

    /** Driver-facing stall message, or null while stage 1 is not being cut. */
    private String stallText() {
        switch (stallGuard.state()) {
            case COOLING: return "STAGE 1 STALL: COOLING, WILL RETRY";
            case LOCKED_OUT: return "STAGE 1 STALL LOCKOUT: CLEAR JAM (shoot, stop/reverse intake or pause resets)";
            default: return null;
        }
    }

    public void addTelemetry(Telemetry telemetry, double now) {
        telemetry.addData("Transfer Stage 1 Device", "%s | %s", HW_TRANSFER_MOTOR, motor.getConnectionInfo());
        telemetry.addData("Transfer Stage 2 Device", "%s | %s", HW_TRANSFER_MOTOR_2, stage2Motor.getConnectionInfo());
        telemetry.addData("Transfer current (A, 5 Hz)", "stage 1 %.2f | stage 2 %.2f",current1,current2);
        telemetry.addData("Transfer Stage 1 Stall Guard", "%s | stalled %.2f / %.2f s | trips %d in a row, %d total"
                        + " | trips below %.0f RPM at %.1f A or more",
                intakeRunning ? stallGuard.state().toString() : "INACTIVE (intake not running forward)",
                stallGuard.stalledSeconds(now), TransferStallGuard.STALL_DETECT_SECONDS,
                stallGuard.consecutiveTrips(), stallGuard.totalTrips(),
                TransferStallGuard.STALL_MAX_RPM, TransferStallGuard.STALL_MIN_CURRENT_AMPS);
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
        telemetry.addData("Transfer Stage 1 Power", "%.2f (%s, open loop)", commandedPower, MOTOR_DIRECTION);
        telemetry.addData("Transfer Stage 1 Speed (RPM, motor encoder)", "%.0f", stage1MeasuredRpm);
        telemetry.addData("Transfer Stage 2 Power", "%.2f (forward shot feed only; off while loading)", stage2Power);
    }
}
