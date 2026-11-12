package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Two-stage transfer on Expansion Hub motor ports 2 and 3; stage 1's encoder input measures turret yaw. */
public class TransferSubsystem {

    public static final String HW_TRANSFER_MOTOR = "transfer_motor_1";
    public static final String HW_TRANSFER_MOTOR_2 = "transfer_motor_2";
    public static final String HW_FULL_SENSOR = "distance_sensor"; // Expansion Hub digital port 0
    public static final DcMotor.Direction MOTOR_DIRECTION = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction STAGE_2_MOTOR_DIRECTION = DcMotor.Direction.FORWARD;
    public static final double TRANSFER_POWER = 1.0;
    public static final double FULL_CONFIRM_TIME_SECONDS = 0.2;
    // After full confirmation stage 1 keeps running this long to seat the fourth ball.
    public static final double STAGE_1_FULL_RUN_ON_SECONDS = 0.5;
    // goBILDA 3124-0001-0001 digital mode: HIGH = object within the configured threshold.
    public static final boolean FULL_SENSOR_ACTIVE_STATE = true;

    private final DcMotorEx motor;
    private final DcMotorEx stage2Motor;
    private final DigitalChannel fullSensor;
    private final TransferController controller = new TransferController(TRANSFER_POWER, FULL_CONFIRM_TIME_SECONDS);
    private boolean sensorState;
    private double current1 = Double.NaN, current2 = Double.NaN, lastCurrentSample = Double.NEGATIVE_INFINITY;
    private double commandedPower = 0.0;
    private double stage2Power = 0.0;
    private boolean feeding = false;
    private boolean loadingPaused = false;
    private boolean forcedStage1Loading = false;

    public TransferSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, HW_TRANSFER_MOTOR);
        motor.setPower(0.0);
        motor.setDirection(MOTOR_DIRECTION);
        // The external turret encoder shares this port; do not reset it or use it to regulate transfer.
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
     * Full is confirmed after 0.2 s of continuous HIGH; stage 1 runs on for 0.5 s to seat the fourth
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
        if (now-lastCurrentSample >= .2) {
            current1 = motor.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
            current2 = stage2Motor.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
            lastCurrentSample = now;
        }
    }

    public void updateCached(double now, boolean feedBalls, boolean pauseLoading) {
        applyOutputs(now, feedBalls, pauseLoading, true);
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
        commandedPower = controller.stage1();
        motor.setPower(commandedPower);
        stage2Power = controller.stage2();
        stage2Motor.setPower(stage2Power);
    }

    public boolean isFull() {
        return controller.full();
    }

    public boolean isSensorHigh() { return sensorState == FULL_SENSOR_ACTIVE_STATE; }
    public boolean isFeeding() { return feeding; }
    public boolean isLoading() { return !feeding && commandedPower > 0; }

    public void pause() {
        commandedPower = stage2Power = 0;
        feeding = false; loadingPaused = true; forcedStage1Loading = false; controller.pause();
        RobotShutdown.stopAll(() -> motor.setPower(0), () -> stage2Motor.setPower(0));
    }

    public void stop() {
        commandedPower = 0.0;
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
        telemetry.addData("Transfer", feeding ? "FEEDING SHOT"
                : isFull() ? "FULL"
                : forcedStage1Loading ? "MANUAL LOADING"
                : loadingPaused ? "PAUSED" : "LOADING");
    }

    /** Put this summary before verbose diagnostics so a stopped transfer has a visible reason. */
    public void addStatusTelemetry(Telemetry telemetry) {
        telemetry.addData("TRANSFER STATUS", "S1 %.2f | S2 %.2f | %s | sensor %s",
                commandedPower, stage2Power,
                feeding ? "FEEDING SHOT"
                        : forcedStage1Loading ? "MANUAL STAGE 1 LOADING"
                        : isFull() && commandedPower > 0 ? "FULL: STAGE 1 RUN-ON (0.5 s)"
                        : isFull() ? "FULL: waiting for LOW or feed"
                        : loadingPaused ? "LOADING PAUSED BY OPMODE" : "LOADING",
                isSensorHigh() ? "HIGH" : "LOW");
    }

    public void addTelemetry(Telemetry telemetry, double now) {
        telemetry.addData("Transfer Stage 1 Device", "%s | %s", HW_TRANSFER_MOTOR, motor.getConnectionInfo());
        telemetry.addData("Transfer Stage 2 Device", "%s | %s", HW_TRANSFER_MOTOR_2, stage2Motor.getConnectionInfo());
        telemetry.addData("Transfer current (A, 5 Hz)", "stage 1 %.2f | stage 2 %.2f",current1,current2);
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
        telemetry.addData("Transfer Stage 1 Power", "%.2f (REVERSE / CW, open loop)", commandedPower);
        telemetry.addData("Transfer Stage 2 Power", "%.2f (forward shot feed only; off while loading)", stage2Power);
    }
}
