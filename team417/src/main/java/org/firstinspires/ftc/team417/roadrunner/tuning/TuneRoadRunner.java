package org.firstinspires.ftc.team417.roadrunner.tuning;

import static com.acmerobotics.roadrunner.Profiles.constantProfile;

import static java.lang.System.nanoTime;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeProfile;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.google.gson.Gson;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team417.roadrunner.Drawing;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;
import org.firstinspires.ftc.team417.roadrunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.team417.roadrunner.TwoDeadWheelLocalizer;

import java.util.ArrayList;
import java.util.List;
import java.util.prefs.Preferences;

/**
 * Math helper for points and vectors:
 * @noinspection unused
 */
class Point { // Can't derive from vector2d because it's marked as final (by default?)
    public double x, y;
    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public Point(Vector2d vector) {
        x = vector.x;
        y = vector.y;
    }
    public Point(VectorF vector) {
        x = vector.get(0);
        y = vector.get(1);
    }
    public Vector2d vector2d() { return new Vector2d(x, y); }
    public Point add(Point other) {
        return new Point(this.x + other.x, this.y + other.y);
    }
    public Point subtract(Point other) {
        return new Point(this.x - other.x, this.y - other.y);
    }
    public Point rotate(double theta) {
        return new Point(Math.cos(theta) * x - Math.sin(theta) * y,
                Math.sin(theta) * x + Math.cos(theta) * y);
    }
    public Point negate() { return new Point(-x, -y); }
    public Point multiply(double scalar) { return new Point(x * scalar, y * scalar); }
    public Point divide(double scalar) { return new Point(x / scalar, y / scalar); }
    public double dot(Point other) {
        return this.x * other.x + this.y * other.y;
    }
    public double cross(Point other) {
        return this.x * other.y - this.y * other.x;
    }
    public double distance(Point other) {
        return Math.hypot(other.x - this.x, other.y - this.y);
    }
    public double length() {
        return Math.hypot(x, y);
    }
    public double atan2() { return Math.atan2(y, x); } // Rise over run
}

/**
 * Class to track encoder ticks.
 * @noinspection UnnecessaryUnicodeEscape
 */
class TickTracker {
    enum Correlation {
        FORWARD, REVERSE, ZERO, NONZERO, POSITIVE, NEGATIVE
    }
    enum Mode {
        FORWARD, LATERAL, ROTATE
    }
    Mode mode;
    IMU imu;
    double lastYaw; // Last yaw measurement
    double totalYaw; // Accumulated yaw

    TickTracker(IMU imu, Mode mode) {
        this.imu = imu;
        this.mode = mode;
        this.lastYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    ArrayList<Counter> counters = new ArrayList<>();
    void register(Encoder encoder, String name, Correlation correlation) {
        counters.add(new Counter(encoder, name, correlation));
    }

    double averageTicks() {
        double tickCount = 0;
        double counterCount = 0;
        for (Counter counter: counters) {
            if ((counter.correlation == Correlation.FORWARD) ||
                (counter.correlation == Correlation.REVERSE)) {
                counterCount++;
                tickCount += Math.abs(counter.ticks());
            }
        }
        return tickCount / counterCount;
    }

    double maxTicks() {
        double maxCount = 0;
        for (Counter counter: counters) {
            maxCount = Math.max(maxCount, Math.abs(counter.ticks()));
        }
        return maxCount;
    }

    boolean report(Telemetry telemetry, String name, String value, String error) {
        boolean passed = true;
        if (!error.isEmpty()) {
            error = ", " + error;
            passed = false;
        }

        String icon = error.isEmpty() ? "\u2705" : "\u274C"; // Green box checkmark, red X
        telemetry.addLine(String.format("%s <b>%s</b>: %s%s", icon, name, value, error));
        return passed;
    }

    @SuppressLint("DefaultLocale")
    boolean reportAll(Telemetry telemetry) {
        final double ZERO_ERROR = 0.05; // Should be no higher than this of the max
        final double STRAIGHT_ERROR = 0.90; // Straight should be no lower than this of the max
        final double ROTATION_ERROR = 0.30; // Rotation should be no lower than this of max

        assert(!counters.isEmpty());
        boolean passed = true;
        double maxTicks = maxTicks();

        if (maxTicks < 300) {
            passed &= report(telemetry, "Ticks so far", String.valueOf(maxTicks), "keep pushing");
        } else {
            if (mode == Mode.ROTATE) {
                double thisYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                double yawDelta = thisYaw - lastYaw;
                lastYaw = thisYaw;

                while (yawDelta < -180)
                    yawDelta += 360.0;
                while (yawDelta >= 180)
                    yawDelta -= 360.0;
                totalYaw += yawDelta;

                String error = "";
                if (Math.abs(totalYaw) < 5) {
                    error = "Yaw too small, are RevHubOrientationOnRobot flags correct when calling imu.initialize()?";
                } else if (totalYaw < -5) {
                    error = "Yaw is negative, you're turning counterclockwise \uD83D\uDD04, right?";
                }
                passed &= report(telemetry, "<b>IMU</b>", String.format("%.1f°", totalYaw), error);
            }

            for (Counter counter : counters) {
                String error = "";
                String value = String.format("%.0f (%.1f%%)",
                        counter.ticks(), 100 * Math.abs(counter.ticks()) / maxTicks);
                switch (counter.correlation) {
                    case FORWARD:
                    case REVERSE:
                        if (mode == Mode.FORWARD) {
                            if (Math.abs(counter.ticks()) < maxTicks * ZERO_ERROR) {
                                error = "is there a bad cable connection?";
                            } else if (counter.ticks() < 0) {
                                error = "set " + counter.name + ".setDirection(DcMotorEx.Direction.REVERSE);";
                            } else if (counter.ticks() < maxTicks * STRAIGHT_ERROR) {
                                error = "too low, does the wheel have bad contact with field?";
                            }
                        } else if (mode == Mode.ROTATE) {
                            if (Math.abs(counter.ticks()) < maxTicks * ZERO_ERROR) {
                                error = "is there a bad cable connection?";
                            } else if ((counter.ticks() < 0) && (counter.correlation == Correlation.FORWARD)) {
                                error = "should be positive, are left & right encoders swapped?";
                            } else if ((counter.ticks() > 0) && (counter.correlation == Correlation.REVERSE)) {
                                error = "should be negative, are left & right encoders swapped?";
                            } else if (Math.abs(counter.ticks()) < maxTicks * ROTATION_ERROR) {
                                error = "too low, does the wheel have bad contact with field?";
                            }
                        }
                        break;
                    case POSITIVE:
                    case NEGATIVE:
                        if (mode == Mode.ROTATE) {
                            if ((counter.ticks() < 0) && (counter.correlation == Correlation.POSITIVE)) {
                                error = "set " + counter.name + ".setDirection(DcMotorEx.Direction.REVERSE);";
                            } else if ((counter.ticks() > 0) && (counter.correlation == Correlation.NEGATIVE)) {
                                error = "set " + counter.name + ".setDirection(DcMotorEx.Direction.REVERSE);";
                            } else if (Math.abs(counter.ticks()) < maxTicks * ZERO_ERROR) {
                                error = "too low, does the wheel have bad contact with field, or "
                                        + "is it too close to center of rotation?";
                            }
                        } else {
                            if (Math.abs(counter.ticks()) < maxTicks * ZERO_ERROR) {
                                error = "too low";
                            } else if ((counter.ticks() < 0) && (counter.correlation == Correlation.POSITIVE)) {
                                error = "should be positive";
                            } else if ((counter.ticks() > 0) && (counter.correlation == Correlation.NEGATIVE)) {
                                error = "should be negative";
                            }
                        }
                        break;
                    case ZERO:
                        if (Math.abs(counter.ticks()) > maxTicks * 0.05) {
                            error = "should be near zero given that it's perpendicular to travel";
                        }
                        break;
                    case NONZERO:
                        if (Math.abs(counter.ticks()) < maxTicks * 0.05) {
                            error = "shouldn't be close to zero";
                        }
                        break;
                }

                passed &= report(telemetry, counter.name, value, error);
            }
        }
        return passed;
    }

    static class Counter {
        Encoder encoder;
        String name;
        Correlation correlation;
        double initialPosition;
        Counter(Encoder encoder, String name, Correlation correlation) {
            this.encoder = encoder;
            this.name = name;
            this.correlation = correlation;
            this.initialPosition = encoder.getPositionAndVelocity().position;
        }
        double ticks() {
            return encoder.getPositionAndVelocity().position - initialPosition;
        }
    }
}

/**
 * Class for remembering all of the tuned settings.
 */
class Settings {
    String robotName;
    TuneRoadRunner.Type type;
    double opticalAngularScalar;
    double opticalLinearScalar;
    SparkFunOTOS.Pose2D opticalOffset = null;

    // Get the settings from the current MecanumDrive object:
    public Settings(MecanumDrive drive) {
        robotName = MecanumDrive.getBotName();
        if (drive.opticalTracker != null) {
            type = TuneRoadRunner.Type.OPTICAL;
            opticalAngularScalar = drive.opticalTracker.getAngularScalar();
            opticalLinearScalar = drive.opticalTracker.getLinearScalar();
            opticalOffset = drive.opticalTracker.getOffset();
        } else if (drive.localizer instanceof MecanumDrive.DriveLocalizer) {
            type = TuneRoadRunner.Type.ALL_WHEEL;
        } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
            type = TuneRoadRunner.Type.THREE_DEAD;
        } else {
            type = TuneRoadRunner.Type.TWO_DEAD;
        }
    }

    // Save the current settings to the preferences database:
    public void save() {
        Preferences preferences = Preferences.userNodeForPackage(Settings.class);
        Gson gson = new Gson();
        String json = gson.toJson(this);

        preferences.put("settings", json);
    }

    // Compare the current settings to the last saved settings. Returns a string of the
    // results; returns null if there is no settings mismatch found.
    public String compareToSaved() {
        // Load the saved settings from the preferences database:
        Preferences preferences = Preferences.userNodeForPackage(Settings.class);
        Gson gson = new Gson();
        String json = preferences.get("settings", "");
        Settings savedSettings = gson.fromJson(json, Settings.class);

        // Now compare to the current settings:
        if (savedSettings == null)
            return null; // No saved settings were found
        if (!savedSettings.robotName.equals(robotName))
            return null; // Different robots, so discard
        if (savedSettings.type != type)
            return null; // Different drive types, so discard
        return null;
    }
}

/** @noinspection UnnecessaryUnicodeEscape*/
@TeleOp
public class TuneRoadRunner extends LinearOpMode {
    enum Type { OPTICAL, ALL_WHEEL, TWO_DEAD, THREE_DEAD }

    // Member fields referenced by every test:
    Ui ui;
    MecanumDrive drive;
    Settings settings;

    // Constants:
    public static int DISTANCE = 72;
    final Pose2d defaultPose = new Pose2d(0, 0, 0);

    // Data structures for the User Interface
    interface MenuStrings {
        String getString(int i);
    }
    /** @noinspection UnnecessaryUnicodeEscape*/
    class Ui {
        // Button press state:
        private final boolean[] buttonPressed = new boolean[5];
        private boolean buttonPress(boolean pressed, int index) {
            boolean press = pressed && !buttonPressed[index];
            buttonPressed[index] = pressed;
            return press;
        }

        // Button press status:
        boolean accept() { return buttonPress(gamepad1.a, 0); }
        boolean cancel() { return buttonPress(gamepad1.b, 1); }
        boolean reposition() { return buttonPress(gamepad1.x, 2); }
        boolean up() { return buttonPress(gamepad1.dpad_up, 3); }
        boolean down() { return buttonPress(gamepad1.dpad_down, 4); }

        // Display the menu:
        /** @noinspection SameParameterValue, StringConcatenationInLoop */
        int menu(String header, int current, boolean topmost, int numStrings, MenuStrings menuStrings) {
            while (opModeIsActive()) {
                String output = "";
                if (up()) {
                    current--;
                    if (current < 0)
                        current = 0;
                }
                if (down()) {
                    current++;
                    if (current == numStrings)
                        current = numStrings - 1;
                }
                if (cancel() && !topmost)
                    return -1;
                if (accept())
                    return current;
                if (header != null) {
                    output += header;
                }
                for (int i = 0; i < numStrings; i++) {
                    String cursor = (i == current) ? "\u27a4" : " ";
                    output += cursor + menuStrings.getString(i) + "\n";
                }
                telemetry.addLine(output);
                telemetry.update();
                // Sleep to allow other system processing (and ironically improve responsiveness):
                sleep(10);
            }
            return topmost ? 0 : -1;
        }

        // Show a message:
        void message(String message) {
            telemetry.addLine(message);
            telemetry.update();
        }

        // Show a message and wait for an A or B button press. If accept (A) is pressed, return
        // success. if cancel (B) is pressed, return failure. The robot CAN be driven while
        // waiting.
        boolean drivePrompt(String message) {
            boolean success = false;
            message(message);
            while (opModeIsActive() && !cancel()) {
                if (accept()) {
                    success = true;
                    break;
                }
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                        -gamepad1.right_stick_x));
                updateSparkFunRotation();
            }
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            return success;
        }

        // Show a message and wait for an A or B button press. If accept (A) is pressed, return
        // success. if cancel (B) is pressed, return failure. The robot CANNOT be driven while
        // waiting.
        boolean prompt(String message) {
            while (opModeIsActive() && !cancel()) {
                message(message);
                if (accept())
                    return true;
            }
            return false;
        }
    }

    // Run an Action but end it early if Cancel is pressed.
    // Returns True if it ran without cancelling, False if it was cancelled.
    private boolean runCancelableAction(Action action) {
        drive.runParallel(action);
        while (opModeIsActive() && !ui.cancel()) {
            TelemetryPacket packet = new TelemetryPacket();
            ui.message("Press B to stop");
            boolean more = drive.doActionsWork(drive.pose, drive.poseVelocity, packet);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            if (!more) {
                // We successfully completed the Action!
                return true; // ====>
            }
        }
        // The user either pressed Cancel or End:
        drive.abortActions();
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
        return false;
    }

    // Road Runner expects the hardware to be in different states when using high-level MecanumDrive/
    // TankDrive functionality vs. its lower-level tuning functionality.
    private void useDrive(boolean enable) {
        DcMotorEx[] motors = { drive.leftFront, drive.leftBack, drive.rightBack, drive.rightFront };
        if (enable) {
            // Initialize hardware state the same way that MecanumDrive does:
            for (DcMotorEx motor: motors) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
        } else {
            // Initialize hardware state in the way that FTC defaults to:
            for (DcMotorEx motor: motors) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
            }
        }
    }

    void encoderPush() {
        useDrive(false); // Don't use MecanumDrive/TankDrive
        boolean passed = false;

        if (ui.prompt("Push the robot forward in a straight line for two or more tiles (24\")."
                + "\n\nPress A to start, B when complete")) {

            TickTracker tracker = new TickTracker(drive.lazyImu.get(), TickTracker.Mode.FORWARD);
            if (drive.localizer instanceof MecanumDrive.DriveLocalizer) {
                MecanumDrive.DriveLocalizer loc = (MecanumDrive.DriveLocalizer) drive.localizer;
                tracker.register(loc.leftFront, "leftFront", TickTracker.Correlation.FORWARD);
                tracker.register(loc.leftBack, "leftBack", TickTracker.Correlation.FORWARD);
                tracker.register(loc.rightBack, "rightBack", TickTracker.Correlation.FORWARD);
                tracker.register(loc.rightFront, "rightFront", TickTracker.Correlation.FORWARD);
            } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
                ThreeDeadWheelLocalizer loc = (ThreeDeadWheelLocalizer) drive.localizer;
                tracker.register(loc.par0, "par0", TickTracker.Correlation.FORWARD);
                tracker.register(loc.par1, "par1", TickTracker.Correlation.FORWARD);
                tracker.register(loc.perp, "perp", TickTracker.Correlation.ZERO);
            } else if (drive.localizer instanceof TwoDeadWheelLocalizer) {
                TwoDeadWheelLocalizer loc = (TwoDeadWheelLocalizer) drive.localizer;
                tracker.register(loc.par, "par", TickTracker.Correlation.FORWARD);
                tracker.register(loc.perp, "perp", TickTracker.Correlation.ZERO);
            }

            while (opModeIsActive() && !ui.cancel()) {
                telemetry.addLine("Push straight forward.\n");
                passed = tracker.reportAll(telemetry);
                if (passed)
                    telemetry.addLine("\nPress B when complete to move to sideways test");
                else
                    telemetry.addLine("\nPress B to cancel");
                telemetry.update();
            }
        }

        if (passed) {
            if (ui.prompt("Push the robot sideways to the left for two or more tiles (24\")."
                    + "\n\nPress A to start, B when complete")) {

                TickTracker tracker = new TickTracker(drive.lazyImu.get(), TickTracker.Mode.LATERAL);
                if (drive.localizer instanceof MecanumDrive.DriveLocalizer) {
                    MecanumDrive.DriveLocalizer loc = (MecanumDrive.DriveLocalizer) drive.localizer;
                    tracker.register(loc.leftFront, "leftFront", TickTracker.Correlation.NEGATIVE);
                    tracker.register(loc.leftBack, "leftBack", TickTracker.Correlation.POSITIVE);
                    tracker.register(loc.rightBack, "rightBack", TickTracker.Correlation.NEGATIVE);
                    tracker.register(loc.rightFront, "rightFront", TickTracker.Correlation.POSITIVE);
                } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
                    ThreeDeadWheelLocalizer loc = (ThreeDeadWheelLocalizer) drive.localizer;
                    tracker.register(loc.par0, "par0", TickTracker.Correlation.ZERO);
                    tracker.register(loc.par1, "par1", TickTracker.Correlation.ZERO);
                    tracker.register(loc.perp, "perp", TickTracker.Correlation.FORWARD);
                } else if (drive.localizer instanceof TwoDeadWheelLocalizer) {
                    TwoDeadWheelLocalizer loc = (TwoDeadWheelLocalizer) drive.localizer;
                    tracker.register(loc.par, "par", TickTracker.Correlation.ZERO);
                    tracker.register(loc.perp, "perp", TickTracker.Correlation.FORWARD);
                }

                while (opModeIsActive() && !ui.cancel()) {
                    telemetry.addLine("Push to the left.\n");
                    passed = tracker.reportAll(telemetry);
                    if (passed)
                        telemetry.addLine("\nPress B when complete");
                    else
                        telemetry.addLine("\nPress B to cancel");
                    telemetry.update();
                }
            }
        }

        if (passed) {
            if (ui.prompt("Rotate the robot counterclockwise at least 90° \uD83D\uDD04 by pushing."
                    + "\n\nPress A to start, B when complete")) {

                TickTracker tracker = new TickTracker(drive.lazyImu.get(), TickTracker.Mode.ROTATE);
                if (drive.localizer instanceof MecanumDrive.DriveLocalizer) {
                    MecanumDrive.DriveLocalizer loc = (MecanumDrive.DriveLocalizer) drive.localizer;
                    tracker.register(loc.leftFront, "leftFront", TickTracker.Correlation.REVERSE);
                    tracker.register(loc.leftBack, "leftBack", TickTracker.Correlation.REVERSE);
                    tracker.register(loc.rightBack, "rightBack", TickTracker.Correlation.FORWARD);
                    tracker.register(loc.rightFront, "rightFront", TickTracker.Correlation.FORWARD);
                } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
                    ThreeDeadWheelLocalizer loc = (ThreeDeadWheelLocalizer) drive.localizer;
                    tracker.register(loc.par0, "par0", TickTracker.Correlation.REVERSE);
                    tracker.register(loc.par1, "par1", TickTracker.Correlation.FORWARD);
                    tracker.register(loc.perp, "perp", TickTracker.Correlation.NONZERO);
                } else if (drive.localizer instanceof TwoDeadWheelLocalizer) {
                    TwoDeadWheelLocalizer loc = (TwoDeadWheelLocalizer) drive.localizer;
                    tracker.register(loc.par, "par", TickTracker.Correlation.REVERSE);
                    tracker.register(loc.perp, "perp", TickTracker.Correlation.NONZERO);
                }

                while (opModeIsActive() && !ui.cancel()) {
                    telemetry.addLine("Rotate counterclockwise\uD83D\uDD04.\n");
                    passed = tracker.reportAll(telemetry);
                    if (passed)
                        telemetry.addLine("\nCongratulations, the encoders and IMU passed! "
                                + "Press B to return to the menu.");
                    else
                        telemetry.addLine("\nPress B to cancel");
                    telemetry.update();
                }
            }
        }
    }

    // Measure the optical linear scale and orientation:
    @SuppressLint("DefaultLocale")
    void opticalLinearScaleAndOrientation() {
        assert(drive.opticalTracker != null);
        useDrive(false); // Don't use MecanumDrive/TankDrive
        String message;

        if (ui.drivePrompt("In this test, you'll push the robot forward in a straight line "
                + "along a field wall for exactly 4 tiles. "
                + "\n\nTo start, align the robot by hand at its starting point. "
                + "\n\nPress A when ready, B to cancel.")) {

            double distance = 0;
            double heading = 0;

            drive.opticalTracker.resetTracking();
            while (opModeIsActive() && !ui.accept()) {
                if (ui.cancel())
                    return; // ====>

                SparkFunOTOS.Pose2D pose = drive.opticalTracker.getPosition();
                distance = Math.hypot(pose.x, pose.y);
                heading = -Math.atan2(pose.y, pose.x); // Rise over run

                message = "Push forward exactly 4 tiles (96\") along a field wall.\n\n";
                message += String.format("&ensp;Sensor reading: (%.1f\", %.1f\", %.1f\u00b0)\n", pose.x, pose.y, Math.toDegrees(pose.h));
                message += String.format("&ensp;Effective distance: %.2f\"\n", distance);
                message += String.format("&ensp;Heading angle: %.2f\u00b0\n", Math.toDegrees(heading));
                message += "\nPress A when complete, B to cancel";
                telemetry.addLine(message);
                telemetry.update();
            }

            // Avoid divide-by-zeroes on aborts:
            if (distance == 0)
                distance = 0.001;

            double oldLinearScalar = drive.opticalTracker.getLinearScalar();
            double newLinearScalar = (96.0 / distance) * oldLinearScalar; // Undo the applied scalar
            double linearScalarChange = Math.abs((oldLinearScalar - newLinearScalar)
                    / oldLinearScalar * 100.0); // Percentage

            SparkFunOTOS.Pose2D offset = drive.opticalTracker.getOffset();
            double oldHeading = offset.h;
            double newHeading = normalizeAngle(heading + oldHeading); // Undo the applied heading
            double headingChange = normalizeAngle(Math.abs(oldHeading - newHeading));

            if ((newLinearScalar < SparkFunOTOS.MIN_SCALAR) || (newLinearScalar > SparkFunOTOS.MAX_SCALAR)) {
                ui.prompt(String.format("The measured distance of %.1f\" is not close enough to "
                        + "the expected distance of 96\". Either something is wrong with the sensor "
                        + "or you didn't push for 4 tiles."
                        + "\n\nAborted, press A to continue.", distance));
            } else {
                message = String.format("New offset heading of %.2f\u00b0 is %.1f\u00b0 off from old.\n", newHeading, Math.toDegrees(headingChange))
                    + String.format("New linear scalar of %.2f is %.1f%% off from old.\n\n", newLinearScalar, linearScalarChange)
                    + "Use these results? Press A if they look good, B to cancel.";
                if (ui.prompt(message)) {
                    settings.opticalLinearScalar = newLinearScalar;
                    settings.opticalOffset.h = newHeading;
                    settings.save();

                    drive.opticalTracker.setLinearScalar(newLinearScalar);
                    drive.opticalTracker.setOffset(new SparkFunOTOS.Pose2D(offset.x, offset.y, newHeading));

                    message = "Go to the configureOtos() routine in MecanumDrive.java (double-tap "
                            + "left-SHIFT and enter 'configureOtos'). Change to these values:\n\n"
                            + String.format("&ensp;headingOffset = %.3f\n", Math.toDegrees(newHeading))
                            + String.format("&ensp;linearScalar = %.3f\n", newLinearScalar)
                            + "\nPress A to continue.";
                    ui.prompt(message);
                }
            }
        }
    }

    // Return a high resolution time count, in seconds:
    public static double time() {
        return nanoTime() * 1e-9;
    }

    // Normalize a radians angle to (-pi, pi]:
    public static double normalizeAngle(double angle) {
        while (angle > Math.PI)
            angle -= 2 * Math.PI;
        while (angle <= -Math.PI)
            angle += 2 * Math.PI;
        return angle;
    }

    // Ramp the motors up or down to or from the target spin speed. Return the amount of
    // rotation
    void rampMotorsSpin(MecanumDrive drive, boolean up) {
        final double RAMP_TIME = 0.5; // Seconds
        final double SPIN_SPEED = 0.2;

        double startTime = time();
        while (opModeIsActive()) {
            double duration = Math.min(time() - startTime, RAMP_TIME);
            double fraction = (up) ? (duration / RAMP_TIME) : (1 - duration / RAMP_TIME);
            drive.rightFront.setPower(fraction * SPIN_SPEED);
            drive.rightBack.setPower(fraction * SPIN_SPEED);
            drive.leftFront.setPower(-fraction * SPIN_SPEED);
            drive.leftBack.setPower(-fraction * SPIN_SPEED);

            updateSparkFunRotation();
            if (duration == RAMP_TIME)
                break; // ===>
        }
    }

    // Structure to describe the center of rotation for the robot:
    static class CenterOfRotation {
        double x;
        double y;
        double farthestPointRadius;
        double traveledRadius;
        double leastSquaresRadius;

        public CenterOfRotation(double x, double y, double farthestPointRadius, double traveledRadius, double leastSquaresRadius) {
            this.x = x;
            this.y = y;
            this.farthestPointRadius = farthestPointRadius;
            this.traveledRadius = traveledRadius;
            this.leastSquaresRadius = leastSquaresRadius;
        }
    }

    // Perform a least-squares fit of an array of points to a circle without using matrices,
    // courtesy of Copilot:
    static CenterOfRotation fitCircle(List<Point> points, double centerX, double centerY) {
        double radius = 0.0;

        // Iteratively refine the center and radius
        for (int iter = 0; iter < 100; iter++) {
            double sumX = 0.0;
            double sumY = 0.0;
            double sumR = 0.0;

            for (Point p : points) {
                double dx = p.x - centerX;
                double dy = p.y - centerY;
                double dist = Math.sqrt(dx * dx + dy * dy);
                sumX += dx / dist;
                sumY += dy / dist;
                sumR += dist;
            }

            centerX += sumX / points.size();
            centerY += sumY / points.size();
            radius = sumR / points.size();
        }

        return new CenterOfRotation(centerX, centerY,0, 0, radius);
    }

    // Persisted state for initiateSparkfunRotation and updateSparkfunRotation:
    double previousSparkFunHeading = 0;
    double accumulatedSparkFunRotation = 0;

    // Start tracking total amount of rotation:
    void initiateSparkFunRotation() {
        assert(drive.opticalTracker != null);
        previousSparkFunHeading = drive.opticalTracker.getPosition().h;
    }

    // Call this regularly to update the tracked amount of rotation:
    SparkFunOTOS.Pose2D updateSparkFunRotation() {
        if (drive.opticalTracker == null) // Handle case where we're using encoders
            return null;

        SparkFunOTOS.Pose2D position = drive.opticalTracker.getPosition();
        accumulatedSparkFunRotation += normalizeAngle(position.h - previousSparkFunHeading);
        previousSparkFunHeading = position.h;
        return position;
    }

    // Get the resulting total rotation amount:
    double getSparkFunRotation() {
        assert(drive.opticalTracker != null);
        updateSparkFunRotation();
        return accumulatedSparkFunRotation;
    }

    // Measure the angular scale and sensor offset:
    @SuppressLint("DefaultLocale")
    void opticalAngularScaleAndOffset() {
        assert(drive.opticalTracker != null);
        final double REVOLUTION_COUNT = 1.0;

        useDrive(true); // Use MecanumDrive/TankDrive
        String results;

        if (!ui.drivePrompt("In this test, you'll position the robot against a wall, then drive "
                + "it out so that the robot can rotate in place 10 times, then position "
                + "the robot against the wall again."
                + "\n\nFirst, carefully drive the robot to a wall and align it so that "
                + "it's facing forward. This marks the start orientation for calibration."
                + "\n\nDrive the robot, press A when ready, B to cancel"))
            return; // ====>

        // Start measuring accumulated rotation:
        initiateSparkFunRotation();

        // Let the user position the robot:
        if (!ui.drivePrompt("Now move the robot far enough away from the wall and any objects so "
                    + "that it can freely rotate in place."
                    + "\n\nPress A when ready for the robot to rotate, B to cancel"))
            return; // ====>

        ui.message(String.format("Rotating %.1f times...", REVOLUTION_COUNT));

        ArrayList<Point> points = new ArrayList<>();
        rampMotorsSpin(drive, true);

        double rotationTarget = REVOLUTION_COUNT * 2 * Math.PI;

        double farthestDistance = 0;
        Point farthestPoint = new Point(0, 0);
        Point currentPoint = new Point(0, 0);
        SparkFunOTOS.Pose2D previousPosition = drive.opticalTracker.getPosition();
        double distanceTraveled = 0;
        double startRotation = getSparkFunRotation();

        // Now rotate the robot:
        while (opModeIsActive()) {
            SparkFunOTOS.Pose2D position = updateSparkFunRotation();
            double rotationAmount = accumulatedSparkFunRotation - startRotation;
            if (rotationAmount >= rotationTarget)
                break; // We're done, break out of this loop!

            currentPoint = new Point(position.x, position.y);
            points.add(currentPoint);
            double distanceFromOrigin = Math.hypot(currentPoint.x, currentPoint.y);
            if (distanceFromOrigin > farthestDistance) {
                farthestDistance = distanceFromOrigin;
                farthestPoint = currentPoint;
            }

            distanceTraveled += Math.hypot(position.x - previousPosition.x, position.y - previousPosition.y);
            previousPosition = position;

            // Update the telemetry:
            double rotationsRemaining = (rotationTarget - rotationAmount) / (2 * Math.PI);
            telemetry.addLine(String.format("%.2f rotations remaining, %d points sampled", rotationsRemaining, points.size()));
            telemetry.addLine("\nPress B to abort.");
            telemetry.update();

            // Draw the circle:
            TelemetryPacket packet = new TelemetryPacket();
            Canvas canvas = packet.fieldOverlay();
            double[] xPoints = new double[points.size()];
            double[] yPoints = new double[points.size()];
            for (int i = 0; i < points.size(); i++) {
                xPoints[i] = points.get(i).x;
                yPoints[i] = points.get(i).y;
            }
            canvas.setStroke("#00ff00");
            canvas.strokePolyline(xPoints, yPoints);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            if (ui.cancel())
                return; // ====>
        }

        // Stop the rotation:
        rampMotorsSpin(drive, false);

        if (!ui.drivePrompt("Now drive the robot to align it at the wall in the same "
                    + "place and orientation as it started."
                    + "\n\nPress A when done, B to cancel"))
            return; // ====>

        CenterOfRotation circleFit = fitCircle(points, farthestPoint.x / 2, farthestPoint.y / 2);

        // Center of rotation results:
        double resultX = farthestPoint.x / 2;
        double resultY = farthestPoint.y / 2;
        double farthestPointRadius = farthestDistance / 2;
        double traveledRadius = distanceTraveled / (2 * REVOLUTION_COUNT * Math.PI);

        // Angular scalar results:
        double totalMeasuredRotation = getSparkFunRotation();
        double totalMeasuredCircles = totalMeasuredRotation / (2 * Math.PI);
        double integerCircles = Math.round(totalMeasuredCircles);
        double angularScalar = integerCircles / totalMeasuredCircles;

        results = String.format("Sensor thinks %.2f circles were completed\n", totalMeasuredCircles);
        results += String.format("Distance traveled: %f, farthestPoint.x: %f, farthestPoint.y: %f, traveledRadius: %f\n",
                distanceTraveled, farthestPoint.x, farthestPoint.y, traveledRadius);
        results += String.format("Offset from center of rotation: (%.2f\", %.2f\"), samples: %d",
                resultX, resultY, points.size());
        results += String.format("Farthest point radius: %.2f, Traveled radius: %.2f", farthestPointRadius, traveledRadius);
        results += String.format("Error (inches): (%.2f, %.2f)\n", currentPoint.x, currentPoint.y);
        results += String.format("Circle-fit position: (%.2f, %.2f), radius: %.2f\n", circleFit.x, circleFit.y, circleFit.leastSquaresRadius);
        results += "\n\n";

        // Do some sanity checking on the results:
        if ((Math.abs(circleFit.x) > 12) || (Math.abs(circleFit.y) > 12)) {
            ui.prompt(results + "The results are bad, the calculated center-of-rotation is bogus.\n\n"
                    + "Aborted, press A to continue.");
            return; // ====>
        }
        if  ((angularScalar < SparkFunOTOS.MIN_SCALAR) || (angularScalar > SparkFunOTOS.MAX_SCALAR)) {
            ui.prompt(results + "The measured number of circles is bad. Did you properly align "
                + "the robot on the wall the same way at both the start and end of this test?\n\n"
                + "Aborted, press A to continue.");
            return; // ====>
        }

        if (ui.prompt(results + "Use these results? Press A if they look good, B to discard them.")) {
            settings.opticalOffset.x = circleFit.x;
            settings.opticalOffset.y = circleFit.y;
            settings.opticalAngularScalar = angularScalar;
            settings.save();

            ui.prompt("Go to the configureOtos() routine in MecanumDrive.java and change these values:\n\n"
                    + String.format("  xOffset = %.2f\n", circleFit.x)
                    + String.format("  yOffset = %.3f\n", circleFit.y)
                    + String.format("  angularScalar = %.3f\n", angularScalar)
                    + "\nPress A to continue.");
        }
    }

    @SuppressLint("DefaultLocale")
    void forwardEncoderTuner() {
        useDrive(false); // Don't use MecanumDrive/TankDrive

        if (ui.prompt("Push the robot forward in a straight line as far as possible. "
                + "Measure distance and set inPerTick = <i>inches-traveled</i> / <i>average-ticks</i>."
                + "\n\nPress A to start, B to cancel")) {

            TickTracker tracker = new TickTracker(drive.lazyImu.get(), TickTracker.Mode.FORWARD);
            if (drive.localizer instanceof MecanumDrive.DriveLocalizer) {
                MecanumDrive.DriveLocalizer loc = (MecanumDrive.DriveLocalizer) drive.localizer;
                tracker.register(loc.leftFront, "leftFront", TickTracker.Correlation.FORWARD);
                tracker.register(loc.leftBack, "leftBack", TickTracker.Correlation.FORWARD);
                tracker.register(loc.rightBack, "rightBack", TickTracker.Correlation.FORWARD);
                tracker.register(loc.rightFront, "rightFront", TickTracker.Correlation.FORWARD);
            } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
                ThreeDeadWheelLocalizer loc = (ThreeDeadWheelLocalizer) drive.localizer;
                tracker.register(loc.par0, "par0", TickTracker.Correlation.FORWARD);
                tracker.register(loc.par1, "par1", TickTracker.Correlation.FORWARD);
                tracker.register(loc.perp, "perp", TickTracker.Correlation.ZERO);
            } else if (drive.localizer instanceof TwoDeadWheelLocalizer) {
                TwoDeadWheelLocalizer loc = (TwoDeadWheelLocalizer) drive.localizer;
                tracker.register(loc.par, "par", TickTracker.Correlation.FORWARD);
                tracker.register(loc.perp, "perp", TickTracker.Correlation.ZERO);
            }

            while (opModeIsActive() && !ui.cancel()) {
                // Report stuff to telemetry:
                boolean passed = tracker.reportAll(telemetry);
                if (passed) {
                    telemetry.addLine(String.format("\n<b>inPerTick</b> = <i>inches-traveled</i> / %.1f",
                            tracker.averageTicks()));
                }
                telemetry.addLine("\nPress B when complete");
                telemetry.update();
            }
        }
    }

    /* @@@
    class LeastSquaresAlgorithm {
        public static void main(String[] args) {
            // Sample data points (x, y)
            double[] xValues = {0, 1, 2, 3};
            double[] yValues = {-1, 0.2, 0.9, 2.1};

            // Calculate the means of x and y
            double xMean = calculateMean(xValues);
            double yMean = calculateMean(yValues);

            // Calculate the sum of (xi - xMean) * (yi - yMean) and (xi - xMean)^2
            double numerator = 0;
            double denominator = 0;
            for (int i = 0; i < xValues.length; i++) {
                double diffX = xValues[i] - xMean;
                double diffY = yValues[i] - yMean;
                numerator += diffX * diffY;
                denominator += diffX * diffX;
            }

            // Calculate the slope (m) and intercept (c)
            double slope = numerator / denominator;
            double intercept = yMean - slope * xMean;

            System.out.println("Slope: " + slope);
            System.out.println("Intercept: " + intercept);
        }

        // Helper method to calculate the mean of an array
        private static double calculateMean(double[] values) {
            double sum = 0;
            for (double value : values) {
                sum += value;
            }
            return sum / values.length;
        }
    }
     */ // @@@

    /**
     * Structure for describing the best-fit-line for a set of points.
     */
    static class BestFitLine {
        double slope;
        double intercept;

        public BestFitLine(double slope, double intercept) {
            this.slope = slope;
            this.intercept = intercept;
        }
    }

    /**
     * Find the best-fit line
     */
    BestFitLine fitLine(ArrayList<Point> points) {
        // Calculate the means of x and y
        Point sum = new Point(0, 0);
        for (Point point: points) {
            sum = sum.add(point);
        }
        Point mean = new Point(sum.x / points.size(), sum.y / points.size());

        // Calculate the sum of (xi - xMean) * (yi - yMean) and (xi - xMean)^2
        double numerator = 0;
        double denominator = 0;
        for (int i = 0; i < points.size(); i++) {
            double diffX = points.get(i).x - mean.x;
            double diffY = points.get(i).y - mean.y;
            numerator += diffX * diffY;
            denominator += diffX * diffX;
        }

        if (denominator == 0)
            // All points are coincident or in a vertical line:
            return new BestFitLine(0, 0);

        // Calculate the slope (m) and intercept (c)
        double slope = numerator / denominator;
        double intercept = mean.y - slope * mean.x;

        return new BestFitLine(slope, intercept);
    }

    // Automatically calculate the kS and kV terms of the feed-forward approximation by
    // ramping up the velocity in a straight line. We increase power by 0.1 each second
    // until it reaches 0.9.
    @SuppressLint("DefaultLocale")
    void autoFeedForwardTuner() {
        final double VOLTAGE_ADDER_PER_SECOND = 0.1;
        final double MAX_VOLTAGE_FACTOR = 0.9;
        final double MAX_SECONDS = MAX_VOLTAGE_FACTOR / VOLTAGE_ADDER_PER_SECOND + 0.1;

        useDrive(false); // Don't use MecanumDrive/TankDrive
        assert(drive.opticalTracker != null);

        if (ui.drivePrompt("Place the robot on the field with as much space in front of it as possible. "
                + "The robot will drive forward in a straight line, starting slowly but getting "
                + "faster and faster. Be ready to press B to stop the robot if it gets close to "
                + "hitting something!"
                + "\n\nDrive the robot to a good spot, press A to start, B to cancel.")) {

            ArrayList<Point> points = new ArrayList<>();
            double startTime = time();
            double oldVoltageFactor = 0;
            double maxVelocity = 0;
            double voltage = drive.voltageSensor.getVoltage();

            // Slowly ramp up the voltage:
            while (opModeIsActive() && !ui.cancel() && ((time() - startTime) < MAX_SECONDS)) {

                // Increase power by 0.1 each second until it reaches 0.9:
                double newVoltageFactor = (time() - startTime) * VOLTAGE_ADDER_PER_SECOND;
                newVoltageFactor = Math.min(newVoltageFactor, MAX_VOLTAGE_FACTOR);

                drive.rightFront.setPower(newVoltageFactor);
                drive.rightBack.setPower(newVoltageFactor);
                drive.leftFront.setPower(newVoltageFactor);
                drive.leftBack.setPower(newVoltageFactor);

                double percentage = newVoltageFactor / MAX_VOLTAGE_FACTOR * 100;
                telemetry.addLine(String.format("%.0f%% done.", percentage));
                telemetry.addLine("\nPress B to abort.");
                telemetry.update();

                SparkFunOTOS.Pose2D velocityVector = drive.opticalTracker.getVelocity();
                double velocity = Math.hypot(velocityVector.x, velocityVector.y);
                points.add(new Point(velocity, oldVoltageFactor));
                maxVelocity = Math.max(velocity, maxVelocity);

                oldVoltageFactor = newVoltageFactor;
            }

            // Stop the robot:
            drive.rightFront.setPower(0);
            drive.rightBack.setPower(0);
            drive.leftFront.setPower(0);
            drive.leftBack.setPower(0);

            if (oldVoltageFactor < MAX_VOLTAGE_FACTOR) {
                ui.prompt("The robot didn't hit top speed before the test was aborted."
                        + "\n\nPress A to continue.");
                return; // ====>
            }
            if (maxVelocity == 0) {
                ui.prompt("The optical tracking sensor returned only zero velocities. "
                        + "Is it working properly?"
                        + "\n\nAborted, press A to continue.");
                return; // ====>
            }

            // Draw the results to the FTC dashboard:
            TelemetryPacket packet = new TelemetryPacket();
            Canvas canvas = packet.fieldOverlay();

            // The canvas coordinates go from -1.0 to 1.0 so scale appropriately:
            double xOffset = -0.9;
            double xScale = 1.8 / maxVelocity;
            double yOffset = -0.9;
            double yScale = 1.8 / MAX_VOLTAGE_FACTOR;

            double[] xPoints = new double[points.size()];
            double[] yPoints = new double[points.size()];
            for (int i = 0; i < points.size(); i++) {
                // Velocity along the x axis, voltage along the y axis:
                xPoints[i] = points.get(i).x * xScale + xOffset;
                yPoints[i] = points.get(i).y * yScale + yOffset;
            }

            canvas.setStroke("#00ff00");
            canvas.strokePolyline(xPoints, yPoints);

            BestFitLine bestFitLine = fitLine(points);

            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            ui.prompt("Check out the graph!"
                    + "\n\nPress A to continue.");
        }
    }

    void driveTest() {
        useDrive(true); // Do use MecanumDrive/TankDrive

        while (opModeIsActive() && !ui.cancel()) {
            // @@@ Make it an exponent!
            // @@@ Add control for specific motors!
            PoseVelocity2d powers = new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                    -gamepad1.right_stick_x);

            drive.setDrivePowers(powers);
            drive.updatePoseEstimate();

            TelemetryPacket p = new TelemetryPacket();
            ui.message("Use the controller to drive the robot around. "
                    + "\n\nPress B when done.");

            Canvas c = p.fieldOverlay();
            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(p);
        }
    }

    @SuppressLint("DefaultLocale")
    void lateralEncoderTuner() {
        useDrive(true); // Do use MecanumDrive/TankDrive

        if (ui.prompt(String.format("The robot will attempt to strafe left for %d inches. "
            + "Measure the actual distance using a tape measure. "
            + "Multiply 'lateralInPerTick' by <distance-measured> / %d."
            + "\n\nPress A to start", DISTANCE, DISTANCE))) {

            runCancelableAction(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(new Vector2d(0, DISTANCE))
                            .build());
        }
    }

    // This is a re-implementation of 'manualFeedforwardTuner' so that DISTANCE can be changed
    // from its hardcoded 64".
    @SuppressLint("DefaultLocale")
    void manualFeedforwardTuner() {
        useDrive(false); // Don't use MecanumDrive/TankDrive

        if (!ui.prompt(String.format("The robot will attempt to drive forwards then backwards for %d inches. "
                + "Tune 'kV' and 'kA' using FTC Dashboard."
                + "\n\nPress A to start, B to stop", DISTANCE)))
            return;

        // Taken from TuningOpModes::register:
        List<Encoder> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();
        List<Encoder> parEncs = new ArrayList<>(), perpEncs = new ArrayList<>();
        if (drive.localizer instanceof MecanumDrive.DriveLocalizer) {
            MecanumDrive.DriveLocalizer dl = (MecanumDrive.DriveLocalizer) drive.localizer;
            leftEncs.add(dl.leftFront);
            leftEncs.add(dl.leftBack);
            rightEncs.add(dl.rightFront);
            rightEncs.add(dl.rightBack);
        } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
            ThreeDeadWheelLocalizer dl = (ThreeDeadWheelLocalizer) drive.localizer;
            parEncs.add(dl.par0);
            parEncs.add(dl.par1);
            perpEncs.add(dl.perp);
        } else if (drive.localizer instanceof TwoDeadWheelLocalizer) {
            TwoDeadWheelLocalizer dl = (TwoDeadWheelLocalizer) drive.localizer;
            parEncs.add(dl.par);
            perpEncs.add(dl.perp);
        } else {
            throw new IllegalArgumentException("unknown localizer: " + drive.localizer.getClass().getName());
        }

        List<Encoder> forwardEncsWrapped = new ArrayList<>();
        forwardEncsWrapped.addAll(leftEncs);
        forwardEncsWrapped.addAll(rightEncs);
        forwardEncsWrapped.addAll(parEncs);

        // Everything below here is taken from ManualFeedforwardTuner::runOpMode():
        TimeProfile profile = new TimeProfile(constantProfile(
                DISTANCE, 0.0,
                MecanumDrive.PARAMS.maxWheelVel,
                MecanumDrive.PARAMS.minProfileAccel,
                MecanumDrive.PARAMS.maxProfileAccel).baseProfile);

        boolean movingForwards = true;
        double startTs = System.nanoTime() / 1e9;

        while (opModeIsActive() && !ui.cancel()) {
            TelemetryPacket packet = new TelemetryPacket();

            for (int i = 0; i < forwardEncsWrapped.size(); i++) {
                int v = forwardEncsWrapped.get(i).getPositionAndVelocity().velocity;
                packet.put(String.format("v%d", i), MecanumDrive.PARAMS.inPerTick * v);
            }

            double ts = System.nanoTime() / 1e9;
            double t = ts - startTs;
            if (t > profile.duration) {
                movingForwards = !movingForwards;
                startTs = ts;
            }

            DualNum<Time> v = profile.get(t).drop(1);
            if (!movingForwards) {
                v = v.unaryMinus();
            }
            packet.put("vref", v.get(0));

            MotorFeedforward feedForward = new MotorFeedforward(MecanumDrive.PARAMS.kS,
                    MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                    MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick);

            double power = feedForward.compute(v) / drive.voltageSensor.getVoltage();
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(power, 0.0), 0.0));

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        // Set power to zero before exiting:
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
    }

    @SuppressLint("DefaultLocale")
    void manualFeedbackTunerAxial() {
        useDrive(true); // Do use MecanumDrive/TankDrive

        if (ui.prompt(String.format("The robot will attempt to drive backwards and forwards for %d inches. "
                + "Tune 'axialGain' so that target and actual align (typical values between 1 and 20)."
                + "\n\nPress A to start, B to stop", DISTANCE))) {

            while (opModeIsActive()) {
                Action action = drive.actionBuilder(new Pose2d(0, 0, 0))
                        .lineToX(DISTANCE)
                        .lineToX(0)
                        .build();
                if (!runCancelableAction(action))
                    return; // Exit when cancelled
            }
        }
    }

    @SuppressLint("DefaultLocale")
    void manualFeedbackTunerLateral() {
        useDrive(true); // Do use MecanumDrive/TankDrive

        if (ui.prompt(String.format("The robot will attempt to strafe left and right for %d inches. "
                + "Tune 'lateralGain' so that target and actual align (typical values between 1 and 20)."
                + "\n\nPress A to start, B to stop", DISTANCE))) {

            while (opModeIsActive()) {
                Action action = drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(0, DISTANCE))
                        .strafeTo(new Vector2d(0, 0))
                        .build();
                if (!runCancelableAction(action))
                    return; // Exit when cancelled
            }
        }
    }

    void manualFeedbackTunerHeading() {
        useDrive(true); // Do use MecanumDrive/TankDrive

        if (ui.prompt("The robot will attempt to rotate in place "
                + "180° clockwise and counterclockwise. "
                + "Tune 'headingGain' so that target and actual align (typical values between 1 and 20)."
                + "\n\nPress A to start, B to stop")) {

            while (opModeIsActive()) {
                Action action = drive.actionBuilder(drive.pose)
                        .turn(Math.PI)
                        .turn(-Math.PI)
                        .build();
                if (!runCancelableAction(action))
                    return; // Exit when cancelled
            }
        }
    }

    void completionTest() {
        useDrive(true); // Do use MecanumDrive/TankDrive

        if (ui.drivePrompt("The robot will drive forward 48 inches using a spline. "
                + "It needs half a tile clearance on either side. "
                + "\n\nDrive the robot to a good spot, press A to start, B to cancel")) {

            telemetry.addLine("Press B to cancel");
            telemetry.update();

            Action action = drive.actionBuilder(drive.pose)
                    .setTangent(Math.toRadians(60))
                    .splineToLinearHeading(new Pose2d(24, 0, Math.toRadians(90)), Math.toRadians(-60))
                    .splineToLinearHeading(new Pose2d(48, 0, Math.toRadians(180)), Math.toRadians(60))
                    .endTrajectory()
                    .setTangent(Math.toRadians(-180))
                    .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(-0.0001)), Math.toRadians(-180))
                    .build();
            runCancelableAction(action);
        }
    }

    // Data structures for building a table of tests:
    interface TestMethod {
        void invoke();
    }
    static class Test {
        TestMethod method;
        String description;
        public Test(TestMethod method, String description) {
            this.method = method;
            this.description = description;
        }
    }

    @Override
    public void runOpMode() {
        // Set the display format to use HTML:
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        // Send telemetry to both FTC Dashboard and the Driver Station:
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize member fields:
        ui = new Ui();
        drive = new MecanumDrive(hardwareMap, defaultPose);
        settings = new Settings(drive);
        String comparison = settings.compareToSaved();
        if (comparison != null) {
            ui.prompt("Did you forget to update your code?\n"
                    + comparison
                    + "\n\nPress B to ignore");
        }

        String configuration = "Mecanum drive, ";
        if (settings.type == Type.OPTICAL) {
            assert(drive.opticalTracker != null);
            configuration += "optical tracking";
            // Set our preferred units:
            drive.opticalTracker.setAngularUnit(AngleUnit.RADIANS);
            drive.opticalTracker.setLinearUnit(DistanceUnit.INCH);
        } else if (settings.type == Type.ALL_WHEEL) {
            configuration += "4 wheel encoders";
        } else if (settings.type == Type.THREE_DEAD) {
            configuration += "3 odometry pods";
        } else {
            configuration += "2 odometry pods";
        }
        String navigation = "Use Dpad to navigate, A to select";
        String heading = String.format("<h4>%s</h4><h2>%s</h2>", configuration, navigation);

        // Dynamically build the list of tests:
        ArrayList<Test> tests = new ArrayList<>();
        tests.add(new Test(this::driveTest, "Drive test (motors)"));
        if (settings.type == Type.OPTICAL) {
            tests.add(new Test(this::opticalLinearScaleAndOrientation, "Optical linear scale & orientation"));
            tests.add(new Test(this::opticalAngularScaleAndOffset, "Optical angular scale & offset"));
            tests.add(new Test(this::autoFeedForwardTuner, "Auto feed forward tuner"));
        } else {
            tests.add(new Test(this::encoderPush, "Push test (encoders and IMU)"));
            tests.add(new Test(this::forwardEncoderTuner, "Forward encoder tuner (inPerTick)"));
            // @@@ Call lateralEncoderTuner only if no dead wheels:
            tests.add(new Test(this::lateralEncoderTuner, "Lateral encoder tuner (lateralInPerTick)"));
        }
        tests.add(new Test(this::manualFeedforwardTuner, "ManualFeedforwardTuner (kV and kA)"));
        tests.add(new Test(this::manualFeedbackTunerAxial, "ManualFeedbackTuner (axialGain)"));
        tests.add(new Test(this::manualFeedbackTunerLateral, "ManualFeedbackTuner (lateralGain)"));
        tests.add(new Test(this::manualFeedbackTunerHeading, "ManualFeedbackTuner (headingGain)"));
        tests.add(new Test(this::completionTest, "Completion test (overall verification)"));

        // Remind the user to press Start on the Driver Station, then press A on the gamepad.
        // We require the latter because enabling the gamepad on the DS after it's been booted
        // causes an A press to be sent to the app, and we don't want that to accidentally
        // invoke a menu option:
        ui.message("<big><big><big><big><big><big><big><big><b>Press \u25B6");
        waitForStart();
        //noinspection StatementWithEmptyBody
        while (opModeIsActive() && !ui.prompt("<big><big><big><big><big><big><big><b>Press Gamepad A"))
            ;

        // Except our main menu loop:
        int selection = 0;
        while (opModeIsActive()) {
            selection = ui.menu(heading, selection, true,
                    tests.size(), i -> tests.get(i).description);

            tests.get(selection).method.invoke();   // Invoke the chosen test
            drive.pose = defaultPose;               // Reset pose for next test
        }
    }
}
