/**
 * Looney Tuner is a parameters tuner for robots using Road Runner.
 */

// @@@ Fix inPerTick
// @@@ Revert changes to return to stock Quick Start code
// @@@ Allow tuners to inherit current OTOS settings
// @@@ Add LED support
// @@@ Add stick support to menus
// @@@ Add max-velocity/max-acceleration testing for both linear and angular
// @@@ Figure out fastLoad for all teams.
// @@@ Show old values, amount of change for: lateralInPerTick
// @@@ Fix rotation gain adjustment
// @@@ Fix bad voltage in spin test

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
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.google.gson.Gson;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;
import com.wilyworks.common.WilyWorks;

import static java.lang.System.out;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team417.roadrunner.Drawing;
import org.firstinspires.ftc.team417.roadrunner.MecanumDrive;

import java.lang.reflect.Field;
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
 * Class for remembering all of the tuned settings.
 * @noinspection AccessStaticViaInstance
 */
class TuneParameters {
    String robotName;
    MecanumDrive.Params params;

    // Get the settings from the current MecanumDrive object:
    public TuneParameters(MecanumDrive drive) {
        robotName = MecanumDrive.getBotName();
        params = drive.PARAMS;
    }

    // Return a deep-copy clone of the current Settings object:
    public TuneParameters createClone() {
        Gson gson = new Gson();
        return gson.fromJson(gson.toJson(this), TuneParameters.class);
    }

    // Save the current settings to the preferences database:
    public void save() {
        Preferences preferences = Preferences.userNodeForPackage(TuneParameters.class);
        Gson gson = new Gson();
        String json = gson.toJson(this);
        preferences.put("settings", json);
    }

    // Compare the saved and current values for a configuration parameter. If they're different,
    // return a string that tells the user
    String comparison = "";
    void compare(String parameter, String format, double oldValue, double newValue) {
        String oldString = String.format(format, oldValue);
        String newString = String.format(format, newValue);
        if (!oldString.equals(newString)) {
            comparison += String.format("&ensp;%s = %s; // Was %s\n", parameter, newString, oldString);
        }
    }
    /** @noinspection SameParameterValue*/
    void compareRadians(String parameter, String format, double oldValue, double newValue) {
        String oldString = String.format(format, Math.toDegrees(oldValue));
        String newString = String.format(format, Math.toDegrees(newValue));
        if (!oldString.equals(newString)) {
            comparison += String.format("&ensp;%s = Math.toDegrees(%s);\n&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;// Was Math.toDegrees(%s)\n", parameter, newString, oldString);
        }
    }

    // Validate that the settings are valid and apply to the current robot:
    TuneParameters getSavedSettings() {
        // Load the saved settings from the preferences database:
        Preferences preferences = Preferences.userNodeForPackage(TuneParameters.class);
        Gson gson = new Gson();
        String json = preferences.get("settings", "");
        TuneParameters savedSettings = gson.fromJson(json, TuneParameters.class);

        // Now compare to the current settings:
        if ((savedSettings == null) || (savedSettings.params == null))
            return null; // No saved settings were found
        if (!savedSettings.robotName.equals(robotName))
            return null; // Different robots, so discard
        return savedSettings;
    }

    // Compare the current settings to the last saved settings. Returns a string that
    // describes how to fix the code if there are any mismatches. It may be an empty string.
    public String compare(TuneParameters oldSettings) {
        comparison = "";

        compare("maxWheelVel", "%.2f", oldSettings.params.maxWheelVel, params.maxWheelVel);
        compare("minProfileAccel", "%.2f", oldSettings.params.minProfileAccel, params.minProfileAccel);
        compare("maxProfileAccel", "%.2f", oldSettings.params.maxProfileAccel, params.maxProfileAccel);
        compare("maxAngVel", "%.2f", oldSettings.params.maxAngVel, params.maxAngVel);
        compare("maxAngAccel", "%.2f", oldSettings.params.maxAngAccel, params.maxAngAccel);
        compare("axialVelGain", "%.2f", oldSettings.params.axialVelGain, params.axialVelGain);
        compare("lateralVelGain", "%.2f", oldSettings.params.lateralVelGain, params.lateralVelGain);
        compare("headingVelGain", "%.2f", oldSettings.params.headingVelGain, params.headingVelGain);
        compare("axialGain", "%.2f", oldSettings.params.axialGain, params.axialGain);
        compare("lateralGain", "%.2f", oldSettings.params.lateralGain, params.lateralGain);
        compare("headingGain", "%.2f", oldSettings.params.headingGain, params.headingGain);
        compare("kS", "%.5f", oldSettings.params.kS, params.kS);
        compare("kV", "%.6f", oldSettings.params.kV, params.kV);
        compare("kA", "%.5f", oldSettings.params.kA, params.kA);
        compare("inPerTick", "%.5f", oldSettings.params.inPerTick, params.inPerTick);
        compare("lateralInPerTick", "%.5f", oldSettings.params.lateralInPerTick, params.lateralInPerTick);
        compare("trackWidthTicks", "%.2f", oldSettings.params.trackWidthTicks, params.trackWidthTicks);
        compare("otos.offset.x", "%.3f", oldSettings.params.otos.offset.x, params.otos.offset.x);
        compare("otos.offset.y", "%.3f", oldSettings.params.otos.offset.y, params.otos.offset.y);
        compareRadians("otos.offset.h", "%.3f", oldSettings.params.otos.offset.h, params.otos.offset.h);
        compare("otos.linearScalar", "%.3f", oldSettings.params.otos.linearScalar, params.otos.linearScalar);
        compare("otos.angularScalar", "%.3f", oldSettings.params.otos.angularScalar, params.otos.angularScalar);
        return comparison;
    }
}

/**
 * Looney Tuner's opMode class for running the tuning tests.
 *
 * @noinspection UnnecessaryUnicodeEscape, AccessStaticViaInstance , ClassEscapesDefinedScope
 */
@SuppressLint("DefaultLocale")
@TeleOp
public class LooneyTuner extends LinearOpMode {
    // Member fields referenced by every test:
    Ui ui;
    MecanumDrive drive;
    TuneParameters parameters;

    // Constants:
    public static int DISTANCE = 72;
    final Pose2d zeroPose = new Pose2d(0, 0, 0);

    // Data structures for the User Interface
    interface MenuStrings {
        String getString(int i);
    }

    /**
     * Class that encapsulates the UI framework.
     *
     * @noinspection UnnecessaryUnicodeEscape
     */
    class Ui {
        // Button press state:
        private final boolean[] buttonPressed = new boolean[10];
        private boolean buttonPress(boolean pressed, int index) {
            boolean press = pressed && !buttonPressed[index];
            buttonPressed[index] = pressed;
            return press;
        }

        // Button press status:
        boolean accept() { return buttonPress(gamepad1.a, 0); }
        boolean cancel() { return buttonPress(gamepad1.b, 1); }
        boolean xButton() { return buttonPress(gamepad1.x, 2); }
        boolean yButton() { return buttonPress(gamepad1.y, 3); }
        boolean dpadUp() { return buttonPress(gamepad1.dpad_up, 4); }
        boolean dpadDown() { return buttonPress(gamepad1.dpad_down, 5); }
        boolean leftBumper() { return buttonPress(gamepad1.left_bumper, 6); }
        boolean rightBumper() { return buttonPress(gamepad1.right_bumper, 7); }
        boolean leftTrigger() { return buttonPress(gamepad1.left_trigger >= 0.5, 8); }
        boolean rightTrigger() { return buttonPress(gamepad1.right_trigger >= 0.5, 9); }

        // Display the menu:
        /** @noinspection SameParameterValue, StringConcatenationInLoop */
        int menu(String header, int current, boolean topmost, int numStrings, MenuStrings menuStrings) {
            while (opModeIsActive()) {
                String output = "";
                if (dpadUp()) {
                    current--;
                    if (current < 0)
                        current = 0;
                }
                if (dpadDown()) {
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
                    if (i == current)
                        output += "<span style='background: #88285a'>\u25c6 " + menuStrings.getString(i) + "</span>\n";
                    else
                        output += "\u25c7 " + menuStrings.getString(i) + "\n";
                }
                telemetryAdd(output);
                telemetryUpdate();
                // Sleep to allow other system processing (and ironically improve responsiveness):
                sleep(10);
            }
            return topmost ? 0 : -1;
        }

        // Show a message:
        void message(String message) {
            telemetryAdd(message);
            telemetryUpdate();
        }

        // Show a message and wait for an A or B button press. If accept (A) is pressed, return
        // success. if cancel (B) is pressed, return failure. The robot CAN be driven while
        // waiting.
        boolean drivePrompt(String message) {
            boolean success = false;
            while (opModeIsActive() && !cancel()) {
                message(message);
                if (accept()) {
                    success = true;
                    break;
                }
                processGamepadDriving();
                updateRotation();
            }
            stopMotors();
            drive.setPose(zeroPose); // Reset the pose once they stopped
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

    // Convert telemetry newlines to HTML breaks so that FTC Dashboard renders the text properly:
    void telemetryAdd(String line) {
        telemetry.addLine(line.replace("\n", "<br>"));
    }

    // Send the telemetry packet to the Driver Station:
    void telemetryUpdate() {
        telemetry.update();
    }

    // Run an Action but end it early if Cancel is pressed.
    // Returns True if it ran without cancelling, False if it was cancelled.
    private boolean runCancelableAction(Action action) {
        drive.runParallel(action);
        while (opModeIsActive() && !ui.cancel()) {
            TelemetryPacket packet = MecanumDrive.getTelemetryPacket();
            ui.message("Press B to stop");
            boolean more = drive.doActionsWork(packet);
            MecanumDrive.sendTelemetryPacket(packet);
            if (!more) {
                // We successfully completed the Action!
                return true; // ====>
            }
        }
        // The user either pressed Cancel or End:
        drive.abortActions();
        stopMotors();
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

    // Shape the stick input for more precision at slow speeds:
    public double shapeStick(double stickValueD) {
        float stickValue = (float) stickValueD;
        // Make slow driving easier on the real robot. Don't bother under Wily Works because
        // then it's too slow:
        float power = WilyWorks.isSimulating ? 1.0f : 2.0f;
        float result = Math.signum(stickValue) * Math.abs((float) Math.pow(stickValue, power));

        // Output spew when weird compiler bug hits because result should never be more than
        // stickValue:
        if (result > stickValue)
            out.printf("raw stick: %.2f, shaped: %.2f, power: %.2f, signum: %.2f\n", stickValue, result, power, Math.signum(stickValue));
        return result;
    }

    // Poll the gamepad input and set the drive motor power accordingly:
    public void processGamepadDriving() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(
            shapeStick(-gamepad1.left_stick_y),
            shapeStick(-gamepad1.left_stick_x)),
            shapeStick(-gamepad1.right_stick_x)));
    }

    // Measure the optical linear scale and orientation:
    void pushTuner() {
        assert(drive.opticalTracker != null);
        useDrive(false); // Don't use MecanumDrive/TankDrive

        // Reset the current OTOS settings:
        Pose2D oldOffset = drive.PARAMS.otos.offset;
        double oldLinearScalar = drive.PARAMS.otos.linearScalar;
        drive.opticalTracker.setOffset(new Pose2D(oldOffset.x, oldOffset.y, 0));
        drive.opticalTracker.setLinearScalar(1.0);

        if (ui.drivePrompt("In this test, you'll push the robot forward in a straight line "
                + "along a field wall for exactly 4 tiles. To start, align the robot by hand "
                + "at its starting point along a wall. "
                + "\n\nPress A when in position, B to cancel.")) {

            double distance = 0;
            double heading = 0;

            drive.opticalTracker.resetTracking();

            boolean success = false;
            while (opModeIsActive() && !ui.cancel()) {
                if (ui.accept()) {
                    success = true;
                    break; // ====>
                }

                Pose2D pose = drive.opticalTracker.getPosition();
                distance = Math.hypot(pose.x, pose.y);
                heading = -Math.atan2(pose.y, pose.x); // Rise over run

                ui.message("Push forward exactly 4 tiles (96\") along a field wall.\n\n"
                    + String.format("&ensp;Sensor reading: (%.1f\", %.1f\", %.1f\u00b0)\n", pose.x, pose.y, Math.toDegrees(pose.h))
                    + String.format("&ensp;Effective distance: %.2f\"\n", distance)
                    + String.format("&ensp;Heading angle: %.2f\u00b0\n", Math.toDegrees(heading))
                    + "\nPress A when you're finished pushing, B to cancel");
            }

            if (success) {
                // Avoid divide-by-zeroes on aborts:
                if (distance == 0)
                    distance = 0.001;

                TuneParameters newParameters = parameters.createClone();
                newParameters.params.otos.linearScalar = (96.0 / distance);
                newParameters.params.otos.offset.h = normalizeAngle(heading);

                double linearScalarChange = Math.abs((oldLinearScalar - newParameters.params.otos.linearScalar)
                        / oldLinearScalar * 100.0); // Percentage
                double headingChange = normalizeAngle(Math.abs(oldOffset.h - newParameters.params.otos.offset.h));

                if (newParameters.params.otos.linearScalar < SparkFunOTOS.MIN_SCALAR) {
                    String message = String.format("The measured distance of %.1f\" is not close enough to "
                            + "the expected distance of 96\". It can't measure more than %.1f\". "
                            + "Either you didn't push straight for 4 tiles or something is wrong "
                            + "with the sensor. ", distance, 96 / SparkFunOTOS.MIN_SCALAR);
                    message += "Maybe the distance of the sensor to the tile is less than 10.0 mm? ";
                    ui.prompt(message + "\n\nAborted, press A to continue");
                } else if (newParameters.params.otos.linearScalar > SparkFunOTOS.MAX_SCALAR) {
                    String message = String.format("The measured distance of %.1f\" is not close enough to "
                            + "the expected distance of 96\". It can't measure less than %.1f\". "
                            + "Either you didn't push straight for 4 tiles or something is wrong "
                            + "with the sensor. ", distance, 96.0 / SparkFunOTOS.MAX_SCALAR);

                    // If the measured distance is close to zero, don't bother with the following
                    // suggestion:
                    if (newParameters.params.otos.linearScalar < 1.5) {
                        message += "Maybe the distance of the sensor to the tile is more than 10.0 mm?";
                    }
                    ui.prompt(message + "\n\nAborted, press A to continue");
                } else {
                    if (ui.prompt(String.format("New offset heading %.3f\u00b0 is %.1f\u00b0 off from old.\n",
                            Math.toDegrees(newParameters.params.otos.offset.h), Math.toDegrees(headingChange))
                            + String.format("New linear scalar %.3f is %.1f%% off from old.\n\n",
                            newParameters.params.otos.linearScalar, linearScalarChange)
                            + "Use these results? Press A if they look good, B to cancel.")) {

                        acceptParameters(newParameters);
                    }
                }
            }
        }

        // Set the hardware to the new (or old) settings:
        setHardware();
    }

    // Prompt the user for how to set the new parameters and save them to the registry:
    public void acceptParameters(TuneParameters newParameters) {
        String comparison = newParameters.compare(parameters);
        if (comparison.isEmpty()) {
            ui.prompt("The results match your current settings.\n\nPress A to continue.");
        } else {
            MecanumDrive.PARAMS = newParameters.params;
            parameters = newParameters;
            parameters.save();
            ui.prompt("Double-tap the shift key in Android Studio, enter 'MD.Params' to jump to the "
                    + "MecanumDrive Params constructor, then update as follows:\n\n"
                    + comparison
                    + "\nPress A to continue.");
        }
    }

    // Set the hardware to the current parameters:
    public void setHardware() {
        drive.configure(hardwareMap);
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

    // Ramp the motors up or down to or from the target spin speed. Turns counter-clockwise
    // when provided a positive power value.
    void rampMotorsSpin(MecanumDrive drive, double targetPower) {
        final double RAMP_TIME = 0.5; // Seconds
        double startPower = drive.rightFront.getPower();
        double deltaPower = targetPower - startPower;
        double startTime = time();
        while (opModeIsActive()) {
            double duration = Math.min(time() - startTime, RAMP_TIME);
            double power = (duration / RAMP_TIME) * deltaPower + startPower;
            drive.rightFront.setPower(power);
            drive.rightBack.setPower(power);
            drive.leftFront.setPower(-power);
            drive.leftBack.setPower(-power);

            updateRotation();
            if (duration == RAMP_TIME)
                break; // ===>
        }
    }

    // Stop all motors:
    void stopMotors() {
        drive.rightFront.setPower(0);
        drive.rightBack.setPower(0);
        drive.leftFront.setPower(0);
        drive.leftBack.setPower(0);
    }

    // Structure to describe the center of rotation for the robot:
    static class Circle {
        double x;
        double y;
        double radius;

        public Circle(double x, double y, double radius) {
            this.x = x;
            this.y = y;
            this.radius = radius;
        }
    }

    // Perform a least-squares fit of an array of points to a circle without using matrices,
    // courtesy of Copilot:
    static Circle fitCircle(List<Point> points, double centerX, double centerY) {
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
        return new Circle(centerX, centerY, radius);
    }

    // Persisted state for initiateSparkFunRotation and updateSparkFunRotation:
    double previousSparkFunHeading = 0;
    double accumulatedSparkFunRotation = 0;

    // Start tracking total amount of rotation:
    double initiateSparkFunRotation() {
        assert(drive.opticalTracker != null);
        previousSparkFunHeading = drive.opticalTracker.getPosition().h;
        return previousSparkFunHeading;
    }

    // Call this regularly to update the tracked amount of rotation:
    void updateRotation() { updateRotationAndGetPose(); }
    Pose2D updateRotationAndGetPose() {
        if (drive.opticalTracker == null) // Handle case where we're using encoders
            return null;

        Pose2D pose = drive.opticalTracker.getPosition();
        accumulatedSparkFunRotation += normalizeAngle(pose.h - previousSparkFunHeading);
        previousSparkFunHeading = pose.h;
        return pose;
    }

    // Get the resulting total rotation amount. You should call updateRotation() before calling
    // this!
    double getSparkFunRotation() {
        return accumulatedSparkFunRotation;
    }

    void drawSpinPoints(ArrayList<Point> points, Circle circle) {
        // Draw the circle on FTC Dashboard:
        TelemetryPacket packet = MecanumDrive.getTelemetryPacket();
        Canvas canvas = packet.fieldOverlay();
        double[] xPoints = new double[points.size()];
        double[] yPoints = new double[points.size()];
        for (int i = 0; i < points.size(); i++) {
            xPoints[i] = points.get(i).x;
            yPoints[i] = points.get(i).y;
        }
        canvas.setStroke("#00ff00");
        canvas.strokePolyline(xPoints, yPoints);

        // Draw the best-fit circle:
        if (circle != null) {
            canvas.setStrokeWidth(1);
            canvas.setStroke("#ff0000");
            canvas.strokeCircle(circle.x, circle.y, circle.radius);
        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    // This is the robot spin test for calibrating the optical sensor angular scale and offset:
    void spinTuner() {
        assert(drive.opticalTracker != null);

        // Number of revolutions to use:
        final double REVOLUTION_COUNT = 10.0;

        // Speed of the revolutions:
        final double SPIN_POWER = 0.5;

        useDrive(true); // Use MecanumDrive/TankDrive

        // Zero these settings for the purpose of this test:
        drive.opticalTracker.setOffset(new Pose2D(0, 0, 0));
        drive.opticalTracker.setAngularScalar(0);

        if (ui.drivePrompt("In this test, you'll position the robot against a wall, then drive "
                + String.format("it out so that the robot can rotate in place %.1f times, then position ", REVOLUTION_COUNT)
                + "the robot against the wall again."
                + "\n\nFirst, carefully drive the robot to a wall and align it so that "
                + "it's facing forward. This marks the start orientation for calibration."
                + "\n\nDrive the robot to the start position, press A when ready, B to cancel")) {

            // Let the user position the robot:
            if (ui.drivePrompt("Now move the robot far enough away from the wall and any objects so "
                    + "that it can freely rotate in place."
                    + "\n\nPress A when ready for the robot to rotate, B to cancel")) {

                ArrayList<Point> points = new ArrayList<>();

                // Spin-up the robot, starting to measure rotation for the 'scalar' computation at
                // this point:
                double scalarStartRotation = initiateSparkFunRotation();
                rampMotorsSpin(drive, SPIN_POWER);

                // Prepare for calculating how far the wheels have traveled:
                double voltage = drive.voltageSensor.getVoltage();
                double wheelVelocity = (SPIN_POWER * voltage - drive.PARAMS.kS) /
                        (drive.PARAMS.kV / drive.PARAMS.inPerTick); // Velocity in inches per second
                double startTime = time();

                out.printf("Velocity: %.2f inches/s\n", wheelVelocity); // @@@

                // Do some preparation for termination:
                final double terminationRotation = REVOLUTION_COUNT * 2 * Math.PI;
                double farthestDistance = 0;
                Point farthestPoint = new Point(0, 0);

                Point originPosition = new Point(0, 0); // The origin of the start of every circle
                double nextCircleRotation = 0;

                Pose2D offsetStartPosition = updateRotationAndGetPose();
                double offsetStartHeading = offsetStartPosition.h;
                double offsetStartRotation = getSparkFunRotation();
                Point rawPosition = new Point(offsetStartPosition.x, offsetStartPosition.y);

                // Now do all of the full-speed spins:
                boolean success = false;
                while (opModeIsActive() && !ui.cancel()) {
                    double offsetRotation = getSparkFunRotation() - offsetStartRotation;

                    // Check if we're at the start of a new circle:
                    if (offsetRotation >= nextCircleRotation) {
                        // Remember the raw position as the start of the new circle:
                        originPosition = rawPosition;
                        nextCircleRotation += 2 * Math.PI;
                    }

                    // Now that we've potentially done the last adjustment, see if we're all done:
                    if (offsetRotation >= terminationRotation) {
                        success = true;
                        break; // ====>
                    }

                    Point currentPoint = rawPosition.subtract(originPosition).rotate(-offsetStartHeading);
                    points.add(currentPoint);
                    double distanceFromOrigin = Math.hypot(currentPoint.x, currentPoint.y);
                    if (distanceFromOrigin > farthestDistance) {
                        farthestDistance = distanceFromOrigin;
                        farthestPoint = currentPoint;
                    }

                    drawSpinPoints(points, null);

                    // Update the telemetry:
                    double rotationsRemaining = (terminationRotation - offsetRotation) / (2 * Math.PI);
                    telemetryAdd(String.format("%.2f rotations remaining, %d samples", rotationsRemaining, points.size()));
                    telemetryAdd("\nPress B to abort.");
                    telemetryUpdate();

                    // Update for next iteration of the loop:
                    Pose2D rawPose = updateRotationAndGetPose();
                    rawPosition = new Point(rawPose.x, rawPose.y);
                }

                double endTime = time();

                // Stop the rotation:
                rampMotorsSpin(drive, 0);

                if ((success) && ui.drivePrompt("Now drive the robot to align it at the wall in the same "
                        + "place and orientation as it started."
                        + "\n\nDrive the robot to its wall position, press A when done, B to cancel")) {

                    Circle circle = fitCircle(points, farthestPoint.x / 2, farthestPoint.y / 2);

                    // Draw results with the fitted circle:
                    drawSpinPoints(points, circle);

                    updateRotationAndGetPose();
                    double totalMeasuredRotation = getSparkFunRotation() - scalarStartRotation;
                    double distancePerRevolution = wheelVelocity * (endTime - startTime) / REVOLUTION_COUNT;
                    processSpinResults(circle, totalMeasuredRotation, distancePerRevolution);
                }
            }
        }

        // Restore the hardware settings:
        setHardware();
    }

    // Process the spin results:
    void processSpinResults(Circle center, double totalMeasuredRotation, double distancePerRevolution) {
        double totalMeasuredCircles = totalMeasuredRotation / (2 * Math.PI);
        double integerCircles = Math.round(totalMeasuredCircles);
        double angularScalar = integerCircles / totalMeasuredCircles;

        // 'Track width' is really the radius of the circle needed to make a complete rotation:
        double trackWidth = distancePerRevolution / (2 * Math.PI);
        double trackWidthTicks = trackWidth / drive.PARAMS.inPerTick;

        out.printf("distancePerRevolution: %.2f\n", distancePerRevolution);
        out.printf("totalMeasuredRotation: %.2f, total circles: %.2f\n", totalMeasuredRotation, totalMeasuredCircles);

        // Undo the offset heading that the OTOS sensor automatically applies:
        Point rawOffset = new Point(center.x, center.y).rotate(-drive.PARAMS.otos.offset.h);

        // Our initial origin where we established (0, 0) at the start of every circle is much
        // less reliable than the best-fit circle center and radius because the former uses
        // only a single sample point while the latter uses all sample points. Consequently, make
        // the offset fit the radius while maintaining the same angle to the center of the circle:
        double theta = Math.atan2(rawOffset.y, rawOffset.x); // Rise-over-run
        Point offset = new Point(Math.cos(theta) * center.radius, Math.sin(theta) * center.radius);

        String results = String.format("Sensor thinks %.2f circles were completed.\n\n", totalMeasuredCircles);
        results += String.format("Circle-fit position: (%.2f, %.2f), radius: %.2f\n", rawOffset.x, rawOffset.y, center.radius);
        results += String.format("Radius-corrected position: (%.2f, %.2f)\n", offset.x, offset.y);
        results += String.format("Angular scalar: %.3f\n", angularScalar);
        results += String.format("Track width: %.2f\"\n", trackWidth);
        results += "\n";

        // Do some sanity checking on the results:
        if ((Math.abs(offset.x) > 12) || (Math.abs(offset.y) > 12)) {
            ui.prompt(results + "The results are bad, the calculated center-of-rotation is bogus.\n\n"
                    + "Aborted, press A to continue.");
        } else if  ((angularScalar < SparkFunOTOS.MIN_SCALAR) || (angularScalar > SparkFunOTOS.MAX_SCALAR)) {
            ui.prompt(results + "The measured number of circles is bad. Did you properly align "
                    + "the robot on the wall the same way at both the start and end of this test?\n\n"
                    + "Aborted, press A to continue.");
        } else {
            TuneParameters newSettings = parameters.createClone();
            newSettings.params.otos.offset.x = offset.x;
            newSettings.params.otos.offset.y = offset.y;
            newSettings.params.otos.angularScalar = angularScalar;
            newSettings.params.trackWidthTicks = trackWidthTicks;

            if (ui.prompt(results + "Use these results? Press A if they look good, B to discard them.")) {
                acceptParameters(newSettings);

                // We changed 'trackWidthTicks' so recreate the kinematics object:
                drive.recreateKinematics();

                out.printf("new trackWidthTicks: %.2f\n", drive.PARAMS.trackWidthTicks); // @@@
            }
        }
    }

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

    // Find the best-fit line.
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
    // ramping up the velocity in a straight line. We increase power by a fixed increment.
    void acceleratingStraightLineTuner() {
        final double VOLTAGE_ADDER_PER_SECOND = 0.3;
        final double MAX_VOLTAGE_FACTOR = 0.9;
        final double MAX_SECONDS = MAX_VOLTAGE_FACTOR / VOLTAGE_ADDER_PER_SECOND + 0.1;

        useDrive(true); // Set the brakes
        assert(drive.opticalTracker != null);

        if (ui.drivePrompt("Drive the robot to a spot on the field with as much space in front of it as possible. "
                + "The robot will drive forward in a straight line, starting slowly but getting "
                + "faster and faster. Be ready to press B to stop the robot when it gets close to "
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
                telemetryAdd(String.format("%.0f%% done.", percentage));
                telemetryAdd("\nPress B to abort.");
                telemetryUpdate();

                Pose2D velocityVector = drive.opticalTracker.getVelocity();
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
            } else if (maxVelocity == 0) {
                ui.prompt("The optical tracking sensor returned only zero velocities. "
                        + "Is it working properly?"
                        + "\n\nAborted, press A to continue.");
            } else {
                // Draw the results to the FTC dashboard:
                TelemetryPacket packet = MecanumDrive.getTelemetryPacket();
                Canvas canvas = packet.fieldOverlay();

                // Set a solid white background:
                canvas.setFill("#ffffff");
                canvas.fillRect(-72, -72, 144, 144);

                // Set the transform:
                canvas.setTranslation(-72, 72);
//                canvas.setScale(144.0 / maxVelocity, 144.0 / MAX_VOLTAGE_FACTOR);

                // The canvas coordinates go from -72 to 72 so scale appropriately:
                double xScale = 140 / maxVelocity;
                double yScale = 140 / MAX_VOLTAGE_FACTOR;

                double[] xPoints = new double[points.size()];
                double[] yPoints = new double[points.size()];
                for (int i = 0; i < points.size(); i++) {
                    // Velocity along the x axis, voltage along the y axis:
                    xPoints[i] = points.get(i).x * xScale;
                    yPoints[i] = points.get(i).y * yScale;
                }

                canvas.setStroke("#00ff00");
                canvas.strokePolyline(xPoints, yPoints);

                BestFitLine bestFitLine = fitLine(points);

                // Draw the best-fit line:
                canvas.setStrokeWidth(1);
                canvas.setStroke("#ff0000");
                canvas.strokeLine(0, bestFitLine.intercept * yScale,
                        200 * xScale, (bestFitLine.intercept + 200 * bestFitLine.slope) * yScale);

                FtcDashboard.getInstance().sendTelemetryPacket(packet);

                out.printf("Intercept: %.3f, Slope: %.3f, Voltage: %.3f\n", bestFitLine.intercept, bestFitLine.slope, voltage);

                TuneParameters newParameters = parameters.createClone();
                newParameters.params.kS = bestFitLine.intercept * voltage;
                newParameters.params.kV = bestFitLine.slope * voltage * parameters.params.inPerTick; // @@@ Normalize?

                if (ui.prompt("Check out the graph on FTC Dashboard!\n\n"
                    + String.format("&ensp;New kS: %.03f, old kS: %.03f\n", newParameters.params.kS, parameters.params.kS)
                    + String.format("&ensp;New kV: %.06f, old kV: %.06f\n", newParameters.params.kV, parameters.params.kV)
                    + "\nIf these look good, press A to accept, B to cancel.")) {

                    acceptParameters(newParameters);
                }
            }
        }

        setHardware();
    }

    // Test the robot motors.
    void wheelDebugger() {
        String[] motorDescriptions = { "leftFront", "leftBack", "rightBack", "rightFront" };
        DcMotorEx[] motors = { drive.leftFront, drive.leftBack, drive.rightBack, drive.rightFront };
        int i = 0;

        stopMotors();
        while (opModeIsActive() && !ui.cancel()) {
            double power = gamepad1.right_trigger - gamepad1.left_trigger;
            String description = motorDescriptions[i];
            telemetryAdd(String.format("This tests every motor individually, now testing '%s'.\n\n", description)
                + String.format("&emsp;%s.setPower(%.2f)\n\n", description, power)
                + "Press right trigger for forward, left trigger for reverse, X for next motor, B to cancel.");
            telemetryUpdate();

            motors[i].setPower(power);
            if (ui.xButton()) {
                motors[i].setPower(0);
                i += 1;
                if (i >= motors.length)
                    i = 0;
            }
        }

        drive.setPose(zeroPose);
        stopMotors();
    }

    // Drive the robot around.
    void driveTest() {
        useDrive(true); // Do use MecanumDrive/TankDrive

        while (opModeIsActive() && !ui.cancel()) {
            if (ui.xButton())
                wheelDebugger();
            if (ui.yButton())
                drive.setPose(zeroPose);

            processGamepadDriving();
            drive.updatePoseEstimate();

            TelemetryPacket p = MecanumDrive.getTelemetryPacket();
            Pose2d pose = drive.pose;
            ui.message("Use the controller to drive the robot around.\n\n"
                    + String.format("&ensp;Pose: (%.2f\", %.2f\", %.2f\u00b0)\n", pose.position.x, pose.position.y, pose.heading.toDouble())
                    + "\nPress X to debug motor wheels, Y to reset pose, B to cancel.");

            Canvas c = p.fieldOverlay();
            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, drive.pose);
            MecanumDrive.sendTelemetryPacket(p);

        }
        stopMotors();
    }

    // Tuner for the lateral multiplier on Mecanum drives.
    void lateralTuner() {
        useDrive(true); // Do use MecanumDrive/TankDrive

        if (ui.drivePrompt(String.format("The robot will strafe left for %d inches (%.1f tiles). ", DISTANCE, DISTANCE / 24.0)
                + "Please ensure that there is sufficient room."
                + "\n\nDrive the robot to position, press A to start, B to cancel")) {

            // Disable the PID gains so that the distance traveled isn't corrected:
            TuneParameters testParameters = parameters.createClone();
            testParameters.params.lateralGain = 0;
            testParameters.params.lateralVelGain = 0;
            testParameters.params.lateralInPerTick = parameters.params.inPerTick;

            // Recreate the Kinematics object based on the new settings:
            drive.recreateKinematics();

            // Point the MecanumDrive to our test PARAMS:
            MecanumDrive.PARAMS = testParameters.params;

            // Drive at a slow speed:
            double maxVelocity = drive.PARAMS.maxWheelVel / 4;

            drive.opticalTracker.setPosition(new Pose2D(0, 0, 0));
            if (runCancelableAction(drive.actionBuilder(drive.pose)
                            .strafeTo(new Vector2d(0, DISTANCE), new TranslationalVelConstraint(maxVelocity))
                            .build())) {

                Pose2D endPose = drive.opticalTracker.getPosition();
                double actualDistance = Math.hypot(endPose.x, endPose.y);
                double lateralMultiplier = actualDistance / DISTANCE;
                double inchesPerTick = parameters.params.inPerTick * lateralMultiplier;

                if (ui.prompt(String.format("Measured lateral multiplier is %.3f (%.5f inches per tick). ", lateralMultiplier, inchesPerTick)
                        + "Does that look good?"
                        + "\n\nPress A to accept, B to cancel")) {
                    TuneParameters newParameters = parameters.createClone();
                    newParameters.params.lateralInPerTick = inchesPerTick;
                    acceptParameters(newParameters);
                }
            }

            // We're done, undo any temporary state we set:
            MecanumDrive.PARAMS = parameters.params;
            drive.recreateKinematics();
        }
    }

    // Tune the kV and kA feed forward parameters:
    void interactiveFeedForwardTuner() {
        useDrive(false); // Don't use MecanumDrive/TankDrive

        // Disable all lateral gains so that backward and forward behavior is not affected by the
        // PID/Ramsete algorithm. It's okay for the axial and rotation gains to be either zero
        // or non-zero:
        TuneParameters testParameters = parameters.createClone();
        testParameters.params.lateralGain = 0;
        testParameters.params.lateralVelGain = 0;
        MecanumDrive.PARAMS = testParameters.params;

        int inputIndex = 0;
        DecimalInput[] inputs = {
            new DecimalInput(drive.PARAMS, "kV", -1, 3, 0.01, 20),
            new DecimalInput(drive.PARAMS, "kA", -4, 5, 0, 1),
        };
        String[] instructions = {
            "Adjust <b>kV</b> to make the horizontal lines as close as possible in height. " +
                "Move the left stick up or down to change <b>kV</b>'s value. " +
                "Remember, <b>kV = vRef / vActual</b>. " +
                "If there are no horizontal lines, decrease the maximum velocity using the left trigger. ",
            "Adjust <b>kA</b> to shift <b>vActual</b> left and right so the angled lines overlap. " +
                "Move the left stick up or down to change <b>kA</b>'s value. ",
        };

        if (ui.drivePrompt(String.format("The robot will constantly drive forwards then backwards for %d inches. "
                + "Tune 'kV' and 'kA' using FTC Dashboard. Follow "
                + "<u><a href='https://learnroadrunner.com/feedforward-tuning.html#tuning'>LearnRoadRunner's guide</a></u>.\n\n"
                + "Make sure %d inches are free in front of the robot to start. "
                + "During the test, you can press Y to override the robot position.\n"
                + "\nPress A to start, B to cancel", DISTANCE, DISTANCE))) {

            // Trigger a reset the first time into the loop:
            boolean movingForwards = false;
            double startTs = 0;
            double maxVelocityFactor = 1.0;
            TimeProfile profile = null;

            while (opModeIsActive() && !ui.cancel()) {
                // Query the new kV or kA value from the user:
                String instruction = instructions[inputIndex]
                        + String.format("Max velocity will be %.0f%% when the next cycle starts.\n",
                            maxVelocityFactor * 100.0);
                inputs[inputIndex].update(instruction,
                        ", Y to reposition, triggers to change max velocity");

                // Update the velocity:
                Pose2D velocity = drive.opticalTracker.getVelocity();
                TelemetryPacket packet = MecanumDrive.getTelemetryPacket();
                packet.put("vActual", velocity.x);

                double ts = time();
                double t = ts - startTs;
                if ((profile == null) || (t > profile.duration)) {
                    movingForwards = !movingForwards;
                    startTs = ts;

                    // Reset the start position on every loop. This ensures that getVelocity().x
                    // is the appropriate velocity to read, and it resets after we reposition
                    // the robot:
                    if (movingForwards) {
                        profile = new TimeProfile(constantProfile(
                                DISTANCE, 0.0,
                                MecanumDrive.PARAMS.maxWheelVel * maxVelocityFactor,
                                MecanumDrive.PARAMS.minProfileAccel,
                                MecanumDrive.PARAMS.maxProfileAccel).baseProfile);
                        drive.setPose(new Pose2d(-DISTANCE / 2.0, 0, 0));
                    }
                }

                DualNum<Time> v = profile.get(t).drop(1);
                if (!movingForwards) {
                    v = v.unaryMinus();
                }
                packet.put("vRef", v.get(0));
                MecanumDrive.sendTelemetryPacket(packet);

                MotorFeedforward feedForward = new MotorFeedforward(MecanumDrive.PARAMS.kS,
                        MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                        MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick);

                double power = feedForward.compute(v) / drive.voltageSensor.getVoltage();
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(power, 0.0), 0.0));

                if (ui.leftTrigger())
                    maxVelocityFactor = Math.max(maxVelocityFactor - 0.1, 0.2);
                if (ui.rightTrigger())
                    maxVelocityFactor = Math.min(maxVelocityFactor + 0.1, 1.0);
                if (ui.xButton())
                    inputIndex ^= 1;
                if (ui.yButton()) {
                    if (!ui.drivePrompt("Drive the robot to reposition it. When you press A, the "
                            + "robot will resume in the forward direction.\n"
                            + "\nPress A when ready to resume, B to cancel."))
                        break; // ====>

                    movingForwards = false;
                    startTs = 0;
                }
                if (ui.accept()) {
                    // Make sure that the user does both kV and kA:
                    if (inputIndex == 0)
                        inputIndex = 1;
                    else {
                        if (ui.prompt("Happy with your results?\n\nPress A to accept, B to cancel"))
                            acceptParameters(testParameters);
                        break; // ====>
                    }
                }
            }
        }

        // We're done, undo any temporary state we set:
        MecanumDrive.PARAMS = parameters.params;
        stopMotors();
    }

    /**
     * Class to handle gamepad input of decimal numbers.
     */
    class DecimalInput {
        final double INITIAL_DELAY = 0.6; // Seconds after initial press before starting to repeat
        final double ADVANCE_DELAY = 0.15; // Seconds after any repeat to repeat again

        Object object; // Object being modified
        String fieldName; // Name of the field in the object being modified
        int digit; // Focus digit (1 is tens, 0 is ones, -1 is tenths, etc.)
        int decimalDigits; // Number of digits to show after the decimal
        String showFormat; // Format string precomputed from decimalDigits
        double minValue; // Clamp ranges
        double maxValue;
        Field field; // Reference to the field being modified
        double value; // Current value
        int lastThumbStick; // Last quantized thumb-stick input (-1, 0 or 1)
        double nextAdvanceTime; // Time at which to advance the value

        // Take a reference to the object and the name of its field to be updated. We use reflection
        // to make the calling code's life a little easier. 'startDigit' dictates the starting
        // focus, 'decimalDigits' is the number of decimal digits to support, 'minValue' and
        // 'maxValue' specify the acceptable ranges.
        DecimalInput(Object object, String fieldName, int startDigit, int decimalDigits, double minValue, double maxValue) {
            this.object = object;
            this.fieldName = fieldName;
            this.digit = startDigit;
            this.decimalDigits = decimalDigits;
            this.showFormat = String.format("%%.%df", decimalDigits);
            this.minValue = minValue;
            this.maxValue = maxValue;

            try {
                field = object.getClass().getDeclaredField(fieldName);
                field.setAccessible(true);
                //noinspection DataFlowIssue
                value = (double) field.get(object);
            } catch (IllegalAccessException|NoSuchFieldException|NullPointerException e) {
                throw new RuntimeException(e);
            }
        }

        // Update the variable according to the latest gamepad input.
        void update(String instructions, String buttonMessage) {
            if (instructions.isEmpty())
                telemetryAdd(String.format("Updating <b>%s</b>. ", fieldName)
                    + "Move the left stick up or down to change its value.\n");
            else
                telemetryAdd(instructions);

            if (ui.leftBumper()) {
                digit = Math.min(digit + 1, 2);
            }
            if (ui.rightBumper()) {
                digit = Math.max(digit - 1, -decimalDigits);
            }
            if ((digit > 0) && (Math.pow(10, digit) > Math.abs(value))) {
                digit--;
            }

            // Advance the value according to the thumb stick state:
            int thumbStick = (int) (gamepad1.left_stick_y / 0.5);
            thumbStick = Math.max(-1, Math.min(1, thumbStick)); // -1, 0 or 1
            if (thumbStick != lastThumbStick) {
                nextAdvanceTime = time() + INITIAL_DELAY;
                lastThumbStick = thumbStick;
                value += lastThumbStick * Math.pow(10, digit);
            } else if (time() > nextAdvanceTime) {
                nextAdvanceTime = time() + ADVANCE_DELAY;
                value += lastThumbStick * Math.pow(10, digit);
            }

            // Clamp new value to acceptable range:
            value = Math.max(minValue, Math.min(maxValue, value));

            // Set the value:
            try {
                field.setDouble(object, value);
            } catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }

            // Show the new value:
            String showValue = String.format(showFormat, value);
            int digitOffset = (digit >= 0) ? -digit - 1 : -digit;
            int digitIndex = showValue.indexOf(".") + digitOffset;
            digitIndex = Math.max(0, Math.min(digitIndex, showValue.length() - 1));
            String prefix = showValue.substring(0, digitIndex);
            String middle = showValue.substring(digitIndex, digitIndex + 1);
            String suffix = showValue.substring(digitIndex + 1);

            // Highlight the focus digit:
            middle = "<span style='background: #88285a'>" + middle + "</span>";

            // Blink the underline every half second:
            if ((((int) (time() * 2)) & 1) != 0) {
                middle = "<u>" + middle + "</u>";
            }

            telemetryAdd(String.format("<big><big>&emsp;%s%s%s</big></big>", prefix, middle, suffix));
            telemetryAdd(String.format("\nPress the bumpers to move the cursor, "
                + "X to switch variables%s, A when done, B to cancel.", buttonMessage));
            telemetryUpdate();
        }
    }

    // Types of interactive PiD tuners:
    enum PidTunerType { AXIAL, LATERAL, HEADING }

    // Adjust the Ramsete/PID values:
    void interactivePidTuner(PidTunerType type) {
        useDrive(true); // Do use MecanumDrive/TankDrive

        TuneParameters testParameters = parameters.createClone();
        MecanumDrive.PARAMS = testParameters.params;
        String prompt;
        String gainName;
        String velGainName;

        TrajectoryActionBuilder trajectory = drive.actionBuilder(zeroPose);
        if (type == PidTunerType.AXIAL) {
            prompt = String.format("The robot will drive backwards and forwards for %d inches. "
                    + "Use FTC Dashboard to tune MecanumDrive.PARAMS.axialGain in the Configuration view "
                    + "so that target and actual align (typical values between 1 and 20).", DISTANCE);
            trajectory = trajectory.lineToX(DISTANCE).lineToX(0);
            gainName = "axialGain";
            velGainName = "axialVelGain";

        } else if (type == PidTunerType.LATERAL) {
            prompt = String.format("The robot will strafe left and right for %d inches. "
                    + "Use FTC Dashboard to tune MecanumDrive.PARAMS.lateralGain in the Configuration view "
                    + "so that target and actual align (typical values between 1 and 20).", DISTANCE);
            trajectory = trajectory.strafeTo(new Vector2d(0, DISTANCE)).strafeTo(new Vector2d(0, 0));
            gainName = "lateralGain";
            velGainName = "lateralVelGain";

        } else {
            prompt = "The robot will rotate in place 180° clockwise and counterclockwise. "
                    + "Use FTC Dashboard to tune MecanumDrive.PARAMS.headingGain in the Configuration view "
                    + "so that target and actual align (typical values between 1 and 20).";
            trajectory = trajectory.turn(Math.PI).turn(-Math.PI);
            gainName = "headingGain";
            velGainName = "headingVelGain";
        }
        if (ui.drivePrompt(prompt + "\n\nPress A to start, B to cancel")) {
            int inputIndex = 0;
            DecimalInput[] inputs = {
                new DecimalInput(drive.PARAMS, gainName, 0, 3, 0, 20),
                new DecimalInput(drive.PARAMS, velGainName, 0, 3, 0, 20),
            };
            while (opModeIsActive() && !ui.cancel()) {
                TelemetryPacket packet = MecanumDrive.getTelemetryPacket();
                inputs[inputIndex].update("", ""); // Update gain variable
                boolean more = drive.doActionsWork(packet); // Drive some more
                MecanumDrive.sendTelemetryPacket(packet);

                // If there is no more actions, start a new one!
                if (!more)
                    drive.runParallel(trajectory.build());

                if (ui.xButton())
                    inputIndex ^= 1; // Toggle the index

                if (ui.accept()) {
                    if (ui.prompt("Happy with your results?\n\nPress A to accept, B to cancel"))
                        acceptParameters(testParameters);
                    break; // ====>
                }
            }
            drive.abortActions();
            stopMotors();
        }
        MecanumDrive.PARAMS = parameters.params;
    }

    // Navigate a short spline as a completion test.
    void completionTest() {
        useDrive(true); // Do use MecanumDrive/TankDrive

        if (ui.drivePrompt("The robot will drive forward 48 inches using a spline. "
                + "It needs half a tile clearance on either side. "
                + "\n\nDrive the robot to a good spot, press A to start, B to cancel")) {

            telemetryAdd("Press B to cancel");
            telemetryUpdate();

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

    // Simple verification test for 'TrackWidth':
    void rotationTest() {
        useDrive(true); // Do use MecanumDrive/TankDrive

        if (ui.drivePrompt("To test 'trackWidthTicks', the robot will turn in-place for two complete "
                + "rotations.\n\nPress A to start, B to cancel.")) {

            // Disable the rotational PID/Ramsete behavior so that we can test just the
            // feed-forward rotation:
            TuneParameters testParameters = parameters.createClone();
            testParameters.params.headingGain = 0;
            testParameters.params.headingVelGain = 0;
            MecanumDrive.PARAMS = testParameters.params;

            out.printf("trackWidthTicks: %.2f\n", MecanumDrive.PARAMS.trackWidthTicks); // @@@

            Action action = drive.actionBuilder(drive.pose)
                    .turn(2 * Math.toRadians(360), new TurnConstraints(
                            MecanumDrive.PARAMS.maxAngVel / 3,
                           -MecanumDrive.PARAMS.maxAngAccel,
                            MecanumDrive.PARAMS.maxAngAccel))
                    .build();
            runCancelableAction(action);

            ui.prompt("The robot should be facing the same direction as when it started. It it's "
                + "not, run the spin tuner again to re-tune 'trackWidthTicks'."
                + "\n\nPress A to continue.");

            // Restore the parameters:
            MecanumDrive.PARAMS = parameters.params;
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
        out.print("417!!!\n"); // @@@@@@@@

        // Set the display format to use HTML:
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        // Send telemetry to both FTC Dashboard and the Driver Station:
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize member fields:
        ui = new Ui();
        drive = new MecanumDrive(hardwareMap, telemetry, gamepad1, zeroPose);
        parameters = new TuneParameters(drive);

        if ((drive.opticalTracker != null) &&
            ((drive.opticalTracker.getAngularUnit() != AngleUnit.RADIANS) ||
             (drive.opticalTracker.getLinearUnit() != DistanceUnit.INCH))) {
            ui.prompt("The SparkFun OTOS must be configured for radians and inches.");
            return; // ====>
        }

        // Dynamically build the list of tests:
        ArrayList<Test> tests = new ArrayList<>();
        tests.add(new Test(this::driveTest, "Drive test (motors)"));
        if (drive.opticalTracker != null) {
            tests.add(new Test(this::pushTuner, "Push tuner (tracking)"));
            tests.add(new Test(this::spinTuner, "Spin tuner (tracking and trackWidthTicks)"));
            tests.add(new Test(this::lateralTuner, "Lateral tuner (lateralInPerTick)"));
            tests.add(new Test(this::acceleratingStraightLineTuner, "Accelerating straight line tuner (kS and kV)"));
            tests.add(new Test(this::interactiveFeedForwardTuner, "Interactive feed forward tuner (kV and kA)"));
            tests.add(new Test(()->interactivePidTuner(PidTunerType.AXIAL), "Interactive PiD tuner (axialGain)"));
            tests.add(new Test(()->interactivePidTuner(PidTunerType.LATERAL), "Interactive PiD tuner (lateralGain)"));
            tests.add(new Test(()->interactivePidTuner(PidTunerType.HEADING), "Interactive PiD tuner (headingGain)"));
            tests.add(new Test(this::rotationTest, "Rotation test (verify trackWidthTicks)"));
            tests.add(new Test(this::completionTest, "Completion test (overall verification)"));
        }

        // Remind the user to press Start on the Driver Station, then press A on the gamepad.
        // We require the latter because enabling the gamepad on the DS after it's been booted
        // causes an A press to be sent to the app, and we don't want that to accidentally
        // invoke a menu option:
        ui.message("<big><big><big><big><big><big><big><big><b>Press \u25B6");
        waitForStart();
        ui.prompt("<big><big><big><big><big><big><big><b>Press Gamepad A or B");

        // Execute our main menu loop:
        int selection = 0;
        while (opModeIsActive()) {
            String heading = "<h2>Use Dpad to navigate, A to select</h2>";
            selection = ui.menu(heading, selection, true,
                    tests.size(), i -> tests.get(i).description);

            tests.get(selection).method.invoke();   // Invoke the chosen test
            drive.setPose(zeroPose);                // Reset pose for next test
        }
    }

    // Check if the robot code setting the MecanumDrive configuration parameters is up to date
    // with the last results from tuning:
    /** @noinspection BusyWait*/
    static public void verifyCodeMatchesTuneResults(MecanumDrive drive, Telemetry telemetry, Gamepad gamepad) {
        // There's no point in complaining about mismatches when running under the simulator:
        if (WilyWorks.isSimulating)
            return; // ====>

        TuneParameters currentSettings = new TuneParameters(drive);
        TuneParameters savedSettings = currentSettings.getSavedSettings();
        if (savedSettings != null) {
            String comparison = savedSettings.compare(currentSettings);
            if (!comparison.isEmpty()) {
                telemetry.clear();
                telemetry.addLine("YOUR CODE IS OUT OF DATE");
                telemetry.addLine();
                telemetry.addLine("The code's configuration parameters don't match the last "
                        + "results saved in LooneyTuner. Double-tap the shift key in Android "
                        + "Studio, enter 'MD.Params' to jump to the MecanumDrive Params constructor, "
                        + "then update as follows:");
                telemetry.addLine();
                telemetry.addLine(comparison);
                telemetry.addLine();
                telemetry.addLine("Please update your code and restart now. Or, to proceed anyway and "
                        + "delete the tuning results, triple-tap the BACK button on the gamepad.");
                telemetry.update();

                // Wait for a triple-tap of the button:
                for (int i = 0; i < 3; i++) {
                    try {
                        while (!gamepad.back)
                            Thread.sleep(1);
                        while (gamepad.back)
                            Thread.sleep(1);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }

                // If we reached this point, the user has chosen to ignore the last tuning results.
                // Override those results with the current settings:
                currentSettings.save();
                telemetry.addLine("Looney Tuner results have been overridden");
                telemetry.update();
            }
        }
    }
}
