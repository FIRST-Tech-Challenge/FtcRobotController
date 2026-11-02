package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.tests.shooter.ShotSpeedTrial;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Locale;

/** Tune the Hub's own velocity PIDF. No drivetrain, automatic firing, or inferred ball counts. */
@Configurable
@TeleOp(name = "TEST - Shooter PIDF Tuner", group = "Testing")
public final class ShooterPidfTuner extends LinearOpMode {
    public static double TARGET_TPS = Shooter.SHOOT_VEL;
    // Replaced by the current Hub coefficients at INIT. Edit AFTER INIT.
    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;
    public static double INTAKE_POWER = Intake.INTAKE_POWER;
    public static double TOLERANCE_TPS = 40;
    public static double READY_DWELL_SECONDS = 0.15;
    public static double SHOT_WINDOW_SECONDS = 2.0;
    public static boolean GATE_INTAKE_ON_READY = false;
    public static double PIDF_STEP = 0.05;
    public static double TARGET_STEP_TPS = 10;

    private static final String[] EDIT_NAMES = {"P", "I", "D", "F", "TARGET_TPS"};
    private int selected;
    private String message = "Hold RB to spin; A marks each ball BEFORE feeding.";
    private String savedPath = "not saved";

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = null;
        DcMotorEx intake = null;
        PIDFCoefficients original = null;
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        try {
            motor = hardwareMap.get(DcMotorEx.class, "shooter");
            motor.setVelocity(0);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            original = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            P = original.p;
            I = original.i;
            D = original.d;
            F = original.f;
            PanelsConfigurables.refreshClass(ShooterPidfTuner.class);
            intake = hardwareMap.tryGet(DcMotorEx.class, "intake");
            if (intake != null) {
                intake.setPower(0);
                intake.setDirection(DcMotorSimple.Direction.REVERSE);
                intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            telemetry.setMsTransmissionInterval(100);
            telemetry.addLine("Shooter PIDF tuner: no drivetrain control.");
            telemetry.addData("Initial Hub PIDF", original);
            telemetry.addData("Intake", intake == null ? "absent: feed manually" : "LB manual feed");
            telemetry.addLine("Panels: edit ShooterPidfTuner AFTER INIT.");
            addControls();
            telemetry.update();
            waitForStart();
            if (isStopRequested()) {
                return;
            }

            Gamepad previous = new Gamepad();
            Gamepad current = new Gamepad();
            previous.copy(gamepad1);
            Settings applied = null;
            ShotSpeedTrial trial = null;
            // Require an intentional release/press if RB was held during START.
            boolean inhibited = gamepad1.right_bumper;
            double lastDisplay = Double.NEGATIVE_INFINITY;
            long startNanos = System.nanoTime();

            while (opModeIsActive()) {
                double now = (System.nanoTime() - startNanos) / 1e9;
                current.copy(gamepad1);
                editFromGamepad(current, previous);
                Settings requested = new Settings();
                String invalid = requested.invalidReason();
                if (current.b || invalid != null) {
                    inhibited = true;
                } else if (!current.right_bumper) {
                    inhibited = false;
                }

                if (invalid == null && !requested.sameAs(applied)) {
                    // Stop intake before a new controller/target; readiness must settle again.
                    if (intake != null) {
                        intake.setPower(0);
                    }
                    motor.setVelocity(0);
                    if (!requested.samePidf(applied)) {
                        motor.setVelocityPIDFCoefficients(requested.p, requested.i, requested.d, requested.f);
                        PIDFCoefficients readback = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                        // Hub coefficients use fixed-point representation, so allow its quantization.
                        if (readback.algorithm != MotorControlAlgorithm.PIDF
                                || !closeCoefficient(readback.p, requested.p)
                                || !closeCoefficient(readback.i, requested.i)
                                || !closeCoefficient(readback.d, requested.d)
                                || !closeCoefficient(readback.f, requested.f)) {
                            throw new IllegalStateException("Shooter PIDF readback did not match; check Hub connection/firmware");
                        }
                    }
                    applied = requested;
                    trial = new ShotSpeedTrial(applied.target, applied.tolerance, applied.dwell, applied.window);
                    message = "Settings applied; trial cleared. Mark four new balls with A.";
                }

                boolean running = invalid == null && !inhibited && current.right_bumper;
                if (running) {
                    motor.setVelocity(applied.target);
                } else {
                    motor.setVelocity(0);
                }
                double speed = motor.getVelocity();
                if (!Double.isFinite(speed)) {
                    motor.setVelocity(0);
                    running = false;
                    inhibited = true;
                    invalid = "Nonfinite encoder velocity; release RB";
                }
                if (trial != null) {
                    if (current.x && !previous.x) {
                        trial.reset();
                        message = "Trial cleared. A marks the next ball BEFORE feeding.";
                    }
                    trial.sample(now, speed, running);
                    if (current.a && !previous.a) {
                        if (!running) {
                            message = "Hold RB before marking a ball.";
                        } else if (trial.mark(now, speed)) {
                            message = "Manual marker " + trial.getShots().size() + "/4; feed this ball now.";
                        } else {
                            message = "Four markers recorded. X starts a new trial.";
                        }
                    }
                }
                boolean ready = running && trial != null && trial.isReady();
                double feed = running && current.left_bumper
                        && (!applied.gate || ready) ? applied.intakePower : 0;
                if (intake != null) {
                    intake.setPower(feed);
                }

                if (current.y && !previous.y) {
                    if (running || current.right_bumper) {
                        message = "Release RB, then press Y to save the finished summary.";
                    } else if (applied != null && trial != null) {
                        try {
                            savedPath = saveSummary(applied, trial, voltage());
                            message = "CSV saved. Copy chosen PIDF into Shooter constants for production.";
                        } catch (IOException e) {
                            message = "CSV save failed: " + e.getMessage();
                        }
                    }
                }

                if (now - lastDisplay >= 0.05) {
                    lastDisplay = now;
                    double volts = voltage();
                    telemetry.addData("State", invalid != null ? invalid
                            : inhibited ? "STOPPED: release RB to re-arm"
                            : ready ? "READY" : running ? "SPINNING / RECOVERING" : "STOPPED");
                    telemetry.addData("Target / actual (ticks/s)", "%.1f / %.1f", requested.target, speed);
                    telemetry.addData("P / I / D / F", "%.4f / %.4f / %.4f / %.4f", requested.p, requested.i, requested.d, requested.f);
                    telemetry.addData("Algorithm", "PIDF (INIT was %s)", original.algorithm);
                    telemetry.addData("Editing", "%s | PIDF step %.4f | target step %.1f", EDIT_NAMES[selected], PIDF_STEP, TARGET_STEP_TPS);
                    telemetry.addData("Feed mode", requested.gate ? "READY gated" : "continuous manual");
                    telemetry.addData("Intake command", intake == null ? "absent" : String.format(Locale.US, "%.2f", feed));
                    telemetry.addData("Battery (V)", volts);
                    if (trial != null) {
                        addTrialTelemetry(trial, applied.target);
                    }
                    telemetry.addLine(message);
                    telemetry.addData("CSV", savedPath);
                    addControls();

                    // Keep numeric fields separate so Panels can graph them.
                    telemetry.addData("targetTps", running ? applied.target : 0);
                    telemetry.addData("velocityTps", speed);
                    telemetry.addData("errorTps", running ? applied.target - speed : 0);
                    telemetry.addData("ready", ready ? 1 : 0);
                    telemetry.addData("intakeCommand", intake == null ? 0 : feed);
                    telemetry.addData("manualMarkers", trial == null ? 0 : trial.getShots().size());
                    telemetry.addData("batteryVolts", volts);
                    telemetry.addData("loopTimeSeconds", now);
                    telemetry.update();
                }
                previous.copy(current);
                sleep(10);
            }
        } finally {
            // Nested finally blocks ensure one failed stop cannot skip the other motor's stop.
            try {
                if (intake != null) {
                    intake.setPower(0);
                }
            } finally {
                try {
                    if (motor != null) {
                        // setPower(0) may be cached after setVelocity(); write an explicit zero target.
                        motor.setVelocity(0);
                    }
                } finally {
                    if (motor != null && original != null) {
                        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, original);
                    }
                }
            }
        }
    }

    private static boolean closeCoefficient(double actual, double requested) {
        return Double.isFinite(actual) && Math.abs(actual - requested) <= 0.001;
    }

    private void editFromGamepad(Gamepad current, Gamepad previous) {
        if (current.dpad_left && !previous.dpad_left) {
            selected = (selected + EDIT_NAMES.length - 1) % EDIT_NAMES.length;
        }
        if (current.dpad_right && !previous.dpad_right) {
            selected = (selected + 1) % EDIT_NAMES.length;
        }
        int sign = (current.dpad_up && !previous.dpad_up ? 1 : 0)
                - (current.dpad_down && !previous.dpad_down ? 1 : 0);
        double step = selected == 4 ? TARGET_STEP_TPS : PIDF_STEP;
        if (sign == 0 || !Double.isFinite(step) || step <= 0) {
            return;
        }
        double delta = sign * step;
        switch (selected) {
            case 0: P = Math.max(0, P + delta); break;
            case 1: I = Math.max(0, I + delta); break;
            case 2: D = Math.max(0, D + delta); break;
            case 3: F = Math.max(0, F + delta); break;
            case 4: TARGET_TPS = Math.max(1, TARGET_TPS + delta); break;
            default: return;
        }
        PanelsConfigurables.refreshClass(ShooterPidfTuner.class);
    }

    private void addControls() {
        telemetry.addLine("RB hold: spin | LB hold: intake | B: stop until RB release");
        telemetry.addLine("A: mark ball BEFORE feed | X: clear trial | Y (stopped): save CSV");
        telemetry.addLine("D-pad left/right: select P/I/D/F/target | up/down: change");
    }

    private void addTrialTelemetry(ShotSpeedTrial trial, double target) {
        telemetry.addData("Manual trial", "%d/4 | %s", trial.getShots().size(), trial.isComplete() ? "finished" : "in progress");
        telemetry.addData("Pre-shot spread (ticks/s)", trial.getPreShotSpread());
        for (ShotSpeedTrial.Shot shot : trial.getShots()) {
            String recovery = Double.isFinite(shot.recoverySeconds)
                    ? String.format(Locale.US, "%.0f ms", shot.recoverySeconds * 1000)
                    : shot.sawDisturbance() ? "not observed" : "no out-of-band dip observed";
            telemetry.addData("Shot " + shot.number,
                    "pre %.0f | min %.0f | drop %.1f%% | over %.0f | recover %s | %s%s",
                    shot.before, shot.minimum, 100 * Math.max(0, target - shot.minimum) / target,
                    Math.max(0, shot.maximum - target), recovery, shot.endReason,
                    shot.readyAtMark ? "" : " | NOT READY at marker");
        }
    }

    private double voltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double value = sensor.getVoltage();
            if (Double.isFinite(value) && value > 0) {
                result = Math.min(result, value);
            }
        }
        return Double.isFinite(result) ? result : Double.NaN;
    }

    private String saveSummary(Settings settings, ShotSpeedTrial trial, double voltage) throws IOException {
        File root = hardwareMap.appContext.getExternalFilesDir(null);
        if (root == null) {
            root = hardwareMap.appContext.getFilesDir();
        }
        File directory = new File(root, "shooter-tuning");
        if (!directory.isDirectory() && !directory.mkdirs()) {
            throw new IOException("Cannot create " + directory);
        }
        File file = new File(directory, "trial-" + System.currentTimeMillis() + ".csv");
        try (PrintWriter writer = new PrintWriter(file, "UTF-8")) {
            writer.println("target_tps,p,i,d,f,tolerance_tps,dwell_s,window_s,intake_power,ready_gate,voltage_at_save");
            writer.printf(Locale.US, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s,%.3f%n",
                    settings.target, settings.p, settings.i, settings.d, settings.f,
                    settings.tolerance, settings.dwell, settings.window, settings.intakePower, settings.gate, voltage);
            writer.println("manual_marker,marker_time_s,ready_at_marker,pre_tps,min_tps,max_tps,target_drop_pct,recovery_s,end_reason");
            for (ShotSpeedTrial.Shot shot : trial.getShots()) {
                writer.printf(Locale.US, "%d,%.6f,%s,%.6f,%.6f,%.6f,%.6f,%s,%s%n",
                        shot.number, shot.markedAt, shot.readyAtMark, shot.before, shot.minimum, shot.maximum,
                        100 * Math.max(0, settings.target - shot.minimum) / settings.target,
                        Double.isFinite(shot.recoverySeconds) ? String.format(Locale.US, "%.6f", shot.recoverySeconds) : "",
                        shot.endReason);
            }
            if (writer.checkError()) {
                throw new IOException("Write failed: " + file);
            }
        }
        return file.getAbsolutePath();
    }

    /** A snapshot prevents telemetry and one trial from using different Panels edits. */
    private static final class Settings {
        final double target = TARGET_TPS;
        final double p = P;
        final double i = I;
        final double d = D;
        final double f = F;
        final double tolerance = TOLERANCE_TPS;
        final double dwell = READY_DWELL_SECONDS;
        final double window = SHOT_WINDOW_SECONDS;
        final double intakePower = INTAKE_POWER;
        final boolean gate = GATE_INTAKE_ON_READY;

        String invalidReason() {
            if (!Double.isFinite(target) || target <= 0 || target > 32767) {
                return "TARGET_TPS must be in (0, 32767]";
            }
            for (double value : new double[]{p, i, d, f}) {
                if (!Double.isFinite(value) || value < 0 || value > 32767) {
                    return "PIDF must be finite and in [0, 32767]";
                }
            }
            if (!Double.isFinite(tolerance) || tolerance <= 0 || tolerance >= target) {
                return "Tolerance must be positive and below target";
            }
            if (!Double.isFinite(dwell) || dwell < 0.05 || dwell > 5
                    || !Double.isFinite(window) || window < dwell || window > 10) {
                return "Dwell: 0.05..5 s; window: dwell..10 s";
            }
            if (!Double.isFinite(intakePower) || Math.abs(intakePower) > 1) {
                return "Intake power must be in [-1, 1]";
            }
            return null;
        }

        boolean samePidf(Settings other) {
            return other != null && p == other.p && i == other.i && d == other.d && f == other.f;
        }

        boolean sameAs(Settings other) {
            return samePidf(other) && target == other.target && tolerance == other.tolerance
                    && dwell == other.dwell && window == other.window
                    && intakePower == other.intakePower && gate == other.gate;
        }
    }
}
