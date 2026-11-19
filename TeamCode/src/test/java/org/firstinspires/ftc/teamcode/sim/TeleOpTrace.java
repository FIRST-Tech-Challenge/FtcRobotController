package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import org.horizon36596.simloop.opmode.OpModePhase;
import org.horizon36596.simloop.opmode.RecordingTelemetry;
import org.horizon36596.simloop.sim.live.GamepadState;
import org.horizon36596.simloop.sim.live.GamepadInputTrack;

/** Primitive decoded data crosses the sim class-loader boundary; retained for both pass and failure. */
final class TeleOpTrace {
    static final class Sample {
        OpModePhase phase;
        int tick;
        double seconds;
        String input, telemetry;
        final double[] powers = new double[4];
        final double[] shaftTicksPerSecond = new double[4];
        double xIn, yIn, headingRad, gatePosition, gatePhysical, gatePulseUs;
        boolean feedAllowed;
        double intakePower, transfer1Power, transfer2Power, flywheelVelocity, yaw, yawCommand, hood, depollinator;
        // Whether stage 1 was in hub velocity control (it never should be).
        boolean transfer1VelocityMode;
        double slidePower, slideTravelDeg, claw;
        boolean distanceHigh;

        String decoded() {
            return phase + "\t" + tick + "\t" + seconds + "\t" + input + "\t"
                    + java.util.Arrays.toString(powers) + "\t" + java.util.Arrays.toString(shaftTicksPerSecond)
                    + "\t" + xIn + "\t" + yIn + "\t" + headingRad + "\t" + gatePosition
                    + "\t" + gatePhysical + "\t" + gatePulseUs + "\t" + feedAllowed
                    + "\t" + intakePower + "\t" + transfer1Power + "\t" + transfer1VelocityMode
                    + "\t" + transfer2Power
                    + "\t" + flywheelVelocity + "\t" + yaw + "\t" + yawCommand + "\t" + hood
                    + "\t" + depollinator + "\t" + slidePower + "\t" + slideTravelDeg + "\t" + claw
                    + "\t" + distanceHigh + "\t" + telemetry;
        }
    }

    final List<Sample> samples = new ArrayList<>();
    final Path directory;
    private String input;
    private OpModePhase phase;
    private int tick;
    private final GamepadInputTrack.Writer track;

    TeleOpTrace(String name, String opMode, double tickSeconds) throws IOException {
        String root = System.getProperty("biobuzz.sim.outputDir");
        if (root == null) throw new IllegalStateException("Run with :TeamCode:simTeleOpTest");
        Files.createDirectories(Paths.get(root));
        directory = Files.createTempDirectory(Paths.get(root), name + "-");
        Files.write(directory.resolve("scenario.txt"), ("OpMode=" + opMode + "\ntickSeconds=" + tickSeconds
                + "\n" + BioBuzzSimModel.PARAMETERS + "\n").getBytes(StandardCharsets.UTF_8));
        java.util.Map<String, String> metadata = new java.util.LinkedHashMap<>();
        metadata.put("opMode", opMode); metadata.put("tickSeconds", Double.toString(tickSeconds));
        track = new GamepadInputTrack.Writer(directory.resolve("gamepads.track"), metadata, false);
    }

    void input(OpModePhase phase, int tick, Gamepad gp1, Gamepad gp2) {
        this.phase = phase;
        this.tick = tick;
        GamepadState one = capture(gp1), two = capture(gp2);
        input = "GP1=" + one + ";GP2=" + two;
        if (phase == OpModePhase.ACTIVE) track.append(new GamepadInputTrack.Frame(tick, one, two));
    }

    private static GamepadState capture(Gamepad pad) {
        java.util.Map<String, Float> axes = new java.util.LinkedHashMap<>();
        java.util.Set<String> buttons = new java.util.LinkedHashSet<>();
        try {
            for (String name : GamepadState.STICK_AXES) axes.put(name, Gamepad.class.getField(name).getFloat(pad));
            for (String name : GamepadState.TRIGGER_AXES) axes.put(name, Gamepad.class.getField(name).getFloat(pad));
            for (String name : GamepadState.BUTTONS) if (Gamepad.class.getField(name).getBoolean(pad)) buttons.add(name);
        } catch (ReflectiveOperationException e) { throw new IllegalStateException(e); }
        return GamepadState.of(axes, buttons);
    }

    void sample(BioBuzzSimHardware hardware, BioBuzzSimModel model, double tickSeconds,
                RecordingTelemetry telemetry) {
        Sample s = new Sample();
        s.phase = phase; s.tick = tick; s.seconds = (samples.size() + 1) * tickSeconds; s.input = input;
        for (int i = 0; i < 4; i++) if (hardware.wheels[i] != null) {
            s.powers[i] = hardware.wheels[i].getPower();
            s.shaftTicksPerSecond[i] = hardware.wheels[i].getPhysicalVelocityTicksPerSecond();
        }
        if (model.drive != null) {
            s.xIn = model.drive.getX(); s.yIn = model.drive.getY(); s.headingRad = model.drive.getHeading();
        }
        if (hardware.gate != null) {
            s.gatePosition = hardware.gate.getPosition(); s.gatePhysical = hardware.gate.getScaledPosition();
            s.gatePulseUs = hardware.gate.getPulseWidth();
        }
        s.telemetry = String.join(" | ", telemetry.lastTransmission());
        if (hardware.intake != null) s.intakePower = hardware.intake.getPower();
        if (hardware.transfer1 != null) {
            s.transfer1Power = hardware.transfer1.getPower();
            s.transfer1VelocityMode = hardware.transfer1.getMode()
                    == com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
        }
        if (hardware.transfer2 != null) s.transfer2Power = hardware.transfer2.getPower();
        if (hardware.flywheel != null) s.flywheelVelocity = hardware.flywheel.getPhysicalVelocityTicksPerSecond();
        if (hardware.yaw1 != null) { s.yawCommand = hardware.yaw1.getPosition(); s.yaw = model.yawDegrees; }
        if (hardware.hood != null) s.hood = hardware.hood.getPosition();
        if (hardware.depollinator != null) s.depollinator = hardware.depollinator.getPosition();
        if (hardware.slideServo != null) {
            s.slidePower = hardware.slideServo.getPower();
            s.slideTravelDeg = model.slideShaftDeg - BioBuzzSimHardware.SLIDE_START_SHAFT_DEG;
        }
        if (hardware.claw != null) s.claw = hardware.claw.getPosition();
        if (hardware.distance != null) s.distanceHigh = hardware.distance.getState();
        s.feedAllowed = telemetry.lastTransmission().contains("Gate Feed Allowed : true");
        samples.add(s);
    }

    Sample at(OpModePhase phase, int tick) {
        return samples.stream().filter(s -> s.phase == phase && s.tick == tick).findFirst()
                .orElseThrow(() -> new AssertionError("Missing " + phase + " tick " + tick + "; trace " + directory));
    }

    List<String> decoded() {
        List<String> rows = new ArrayList<>();
        for (Sample s : samples) rows.add(s.decoded());
        return rows;
    }

    void save(String outcome) throws IOException {
        List<String> rows = new ArrayList<>();
        rows.add("phase\ttick\tseconds\tinput\twheelPowers\tshaftTicksPerSecond\txIn\tyIn\theadingRad"
                + "\tgateLogical\tgatePhysical\tgatePulseUs\tfeedAllowed\tintakePower\ttransfer1Power\ttransfer1VelocityMode\ttransfer2Power"
                + "\tflywheelTicksPerSecond\tyawDegrees\tyawServoCommand\thoodServo\tdepollinatorServo\tslidePower\tslideTravelDeg\tclawServo\tdistanceHigh\ttelemetry");
        rows.addAll(decoded());
        Files.write(directory.resolve("trace.tsv"), rows, StandardCharsets.UTF_8);
        track.close();
        Files.write(directory.resolve("runner-outcome.txt"), (outcome + "\nLast requested phase=" + phase
                + ",tick=" + tick + ",input=" + input + "\nAssertion results: see Gradle JUnit XML/HTML reports.\n")
                .getBytes(StandardCharsets.UTF_8));
    }
}
