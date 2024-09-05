/**
 * Looney Tuner is a parameters tuner for robots using Road Runner.
 */

// Short-term:
// @@@ No preview in Wily Works with completion test
// @@@ Fix ramp distance bug/feature
// @@@ Show old values, amount of change for: lateralInPerTick
// @@@ Add velocity test to extras
// @@@ Figure out fastLoad for all teams
//
// Long-term:
// @@@ Do something about inPerTick
// @@@ Revert changes to return to stock Quick Start code
// @@@ Add LED support
// @@@ Add max-velocity/max-acceleration testing for both linear and angular

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
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
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
    boolean useHtml = false;
    String comparison = "";
    void compare(String parameter, String format, double oldValue, double newValue) {
        String oldString = String.format(format, oldValue);
        String newString = String.format(format, newValue);
        if (!oldString.equals(newString)) {
            comparison += (useHtml) ? "&ensp;" : "    ";
            comparison += String.format("%s = %s; // Was %s\n", parameter, newString, oldString);
        }
    }
    /** @noinspection SameParameterValue*/
    void compareRadians(String parameter, String format, double oldValue, double newValue) {
        String oldString = String.format(format, Math.toDegrees(oldValue));
        String newString = String.format(format, Math.toDegrees(newValue));
        if (!oldString.equals(newString)) {
            comparison += (useHtml) ? "&ensp;" : "    ";
            comparison += String.format("%s = Math.toRadians(%s);\n", parameter, newString);
            comparison += (useHtml) ? "&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;" : "                          ";
            comparison += String.format("// Was Math.toRadians(%s)\n", oldString);
        }
    }

    // Validate that the settings are valid and apply to the current robot:
    TuneParameters getSavedParameters() {
        // Load the saved settings from the preferences database:
        Preferences preferences = Preferences.userNodeForPackage(TuneParameters.class);
        Gson gson = new Gson();
        String json = preferences.get("settings", "");
        TuneParameters savedParameters = gson.fromJson(json, TuneParameters.class);

        // Now compare to the current settings:
        if ((savedParameters == null) || (savedParameters.params == null))
            return null; // No saved settings were found
        if (!savedParameters.robotName.equals(robotName))
            return null; // Different robots, so discard
        return savedParameters;
    }

    // Compare the current settings to the last saved settings. Returns a string that
    // describes how to fix the code if there are any mismatches. It may be an empty string.
    public String compare(TuneParameters oldSettings, boolean useHtml) {
        this.useHtml = useHtml;
        this.comparison = "";

        compare("inPerTick", "%.5f", oldSettings.params.inPerTick, params.inPerTick);
        compare("lateralInPerTick", "%.5f", oldSettings.params.lateralInPerTick, params.lateralInPerTick);
        compare("trackWidthTicks", "%.2f", oldSettings.params.trackWidthTicks, params.trackWidthTicks);
        compare("kS", "%.5f", oldSettings.params.kS, params.kS);
        compare("kV", "%.6f", oldSettings.params.kV, params.kV);
        compare("kA", "%.5f", oldSettings.params.kA, params.kA);
        compare("axialGain", "%.2f", oldSettings.params.axialGain, params.axialGain);
        compare("axialVelGain", "%.2f", oldSettings.params.axialVelGain, params.axialVelGain);
        compare("lateralGain", "%.2f", oldSettings.params.lateralGain, params.lateralGain);
        compare("lateralVelGain", "%.2f", oldSettings.params.lateralVelGain, params.lateralVelGain);
        compare("headingGain", "%.2f", oldSettings.params.headingGain, params.headingGain);
        compare("headingVelGain", "%.2f", oldSettings.params.headingVelGain, params.headingVelGain);
        compare("otos.offset.x", "%.3f", oldSettings.params.otos.offset.x, params.otos.offset.x);
        compare("otos.offset.y", "%.3f", oldSettings.params.otos.offset.y, params.otos.offset.y);
        compareRadians("otos.offset.h", "%.3f", oldSettings.params.otos.offset.h, params.otos.offset.h);
        compare("otos.linearScalar", "%.3f", oldSettings.params.otos.linearScalar, params.otos.linearScalar);
        compare("otos.angularScalar", "%.3f", oldSettings.params.otos.angularScalar, params.otos.angularScalar);
        compare("maxWheelVel", "%.2f", oldSettings.params.maxWheelVel, params.maxWheelVel);
        compare("minProfileAccel", "%.2f", oldSettings.params.minProfileAccel, params.minProfileAccel);
        compare("maxProfileAccel", "%.2f", oldSettings.params.maxProfileAccel, params.maxProfileAccel);
        compare("maxAngVel", "%.2f", oldSettings.params.maxAngVel, params.maxAngVel);
        compare("maxAngAccel", "%.2f", oldSettings.params.maxAngAccel, params.maxAngAccel);
        return comparison;
    }
}

/**
 * Class for encapsulating all Gui operations.
 * @noinspection StringConcatenationInsideStringBufferAppend, UnnecessaryUnicodeEscape, unused
 */
class Gui {
    static final double ANALOG_THRESHOLD = 0.5; // Threshold to consider an analog button pressed
    static private final String DESCRIPTOR_SEPARATOR = "::";
    static final double INITIAL_DELAY = 0.6; // Seconds after initial press before starting to repeat
    static final double ADVANCE_DELAY = 0.1; // Seconds after any repeat to repeat again
    private static Gui gui; // Points to our own  singleton object
    Gamepad gamepad; // Gamepad to use for control
    ArrayList<Widget> menuStack = new ArrayList<>(); // Stack of menus, the last is the current
    int lastInput; // Last quantized input (-1, 0 or 1)
    double nextAdvanceTime; // Time at which to advance the value

    abstract private static class Widget {
        String description;
        BooleanSupplier isEnabled;
        Widget(String descriptor) {
            // The description comes after the last separator:
            int lastIndex = descriptor.lastIndexOf(DESCRIPTOR_SEPARATOR);
            if (lastIndex != -1) {
                descriptor = descriptor.substring(lastIndex + DESCRIPTOR_SEPARATOR.length());
            }
            description = descriptor;
            isEnabled = ()->true; // Default is to be enabled
        }
        abstract public String string();
    }
    private static class MenuWidget extends Widget {
        ArrayList<Widget> widgets = new ArrayList<>(); // List of widgets in this menu
        int current; // Index in widgets that has the UI focus
        public MenuWidget(String descriptor) {
            super(descriptor);
        }
        public String string() {
            return description + "..."; // "\uD83D\uDCC1 is Folder symbol
        }
    }
    private static class ToggleWidget extends Widget {
        boolean value;
        Consumer<Boolean> callback;
        public ToggleWidget(String descriptor, boolean value, Consumer<Boolean> callback) {
            super(descriptor); this.value = value; this.callback = callback;
        }
        public String string() {
            return (value ? "\u2612" : "\u2610") + " " + description; // checked-box, empty box
        }
    }
    private static class ListWidget extends Widget {
        int index;
        String[] list;
        BiConsumer<Integer, String> callback;
        public ListWidget(String descriptor, int index, String[] list, BiConsumer<Integer, String> callback) {
            super(descriptor); this.index = index; this.list = list; this.callback = callback;
        }
        public String string() {
            return "\u2194\uFE0F <b>" + list[index] + "</b>: " + description; // blue left/right arrow
        }
    }
    private static class ActivationWidget extends Widget {
        Function<Boolean, String> callback;
        public ActivationWidget(String descriptor, Function<Boolean, String> callback) {
            super(descriptor); this.callback = callback;
        }
        public String string() { return "\u2757 " + callback.apply(false); } // red exclamation mark
    }
    private static class StatsWidget extends Widget {
        Supplier<String> callback;
        public StatsWidget(String descriptor, Supplier<String> callback) {
            super(descriptor); this.callback = callback;
        }
        public String string() { return "\uD83D\uDCCA " + description + "..."; }
    }
    private static class RunWidget extends Widget {
        Runnable runnable;
        public RunWidget(String descriptor, Runnable runnable, BooleanSupplier isEnabled) {
            super(descriptor);
            this.runnable = runnable;
            if (isEnabled != null)
                this.isEnabled = isEnabled;
        }
        public String string() {
            return isEnabled.getAsBoolean() ? description : "<font color='#808080'>" + description + "</font>";
        }
    }

    // Button press state:
    private final boolean[] buttonPressed = new boolean[10];
    private boolean buttonPress(boolean pressed, int index) {
        boolean press = pressed && !buttonPressed[index];
        buttonPressed[index] = pressed;
        return press;
    }

    // Button press status:
    boolean accept() { return buttonPress(gamepad.a, 0); }
    boolean cancel() { return buttonPress(gamepad.b, 1); }
    boolean xButton() { return buttonPress(gamepad.x, 2); }
    boolean yButton() { return buttonPress(gamepad.y, 3); }
    boolean up() { return buttonPress(gamepad.dpad_up, 4); }
    boolean down() { return buttonPress(gamepad.dpad_down, 5); }
    boolean left() { return buttonPress(gamepad.dpad_left, 6); }
    boolean right() { return buttonPress(gamepad.dpad_right, 7); }
    boolean leftTrigger() { return buttonPress(gamepad.left_trigger >= ANALOG_THRESHOLD, 8); }
    boolean rightTrigger() { return buttonPress(gamepad.right_trigger >= ANALOG_THRESHOLD, 9); }

    // Constructor:
    public Gui(Gamepad gamepad) {
        Gui.gui = this;
        this.gamepad = gamepad;

        // Create the root menu:
        menuStack.add(new MenuWidget(""));
    }

    // Return a high resolution time count, in seconds:
    public static double time() {
        return nanoTime() * 1e-9;
    }

    // Update loop for the Gui.
    String update() {
        StringBuilder output = new StringBuilder();

        // Add a header with submenu names:
        output.append("<h2>");
        if (menuStack.size() <= 1) {
            output.append("Dpad to navigate, "+LooneyTuner.A+" to select");
        } else {
            for (int i = 1; i < menuStack.size(); i++) {
                if (i > 1)
                    output.append("\u00b7");
                output.append(menuStack.get(i).description);
            }
            output.append(", "+LooneyTuner.A+" to select, "+LooneyTuner.B+" to exit");
        }
        output.append("</h2>");

        // Process dpad up and down with auto-repeat and clamping:
        MenuWidget menu = (MenuWidget) menuStack.get(menuStack.size() - 1);
        int input = gamepad.dpad_up ? -1 : (gamepad.dpad_down ? 1 : 0); // -1, 0 or 1
        if (input != lastInput) {
            nextAdvanceTime = time() + INITIAL_DELAY;
            lastInput = input;
            menu.current += lastInput;
        } else if (time() > nextAdvanceTime) {
            nextAdvanceTime = time() + ADVANCE_DELAY;
            menu.current += lastInput;
        }
        menu.current = Math.max(0, Math.min(menu.widgets.size() - 1, menu.current));

        // Now output the widgets:
        for (int i = 0; i < menu.widgets.size(); i++) {
            Widget widget = menu.widgets.get(i);
            if (i != menu.current)
                output.append("\u25c7 " + widget.string() + "\n");
            else
                // Highlight current item:
                output.append("<span style='background: #88285a'>\u25c6 " + widget.string() + "</span>\n");
        }

        Widget widget = menu.widgets.get(menu.current);
        if (cancel()) {
            if (menuStack.size() > 1) {
                // Pop up the menu stack:
                menuStack.remove(menuStack.size() - 1);
            }
        }
        else if (widget instanceof ToggleWidget) {
            ToggleWidget toggleWidget = (ToggleWidget) widget;
            if (accept()) {
                toggleWidget.value = !toggleWidget.value;
                toggleWidget.callback.accept(toggleWidget.value);
            }
        } else if (widget instanceof ListWidget) {
            ListWidget listWidget = (ListWidget) widget;
            boolean left = left();
            boolean right = right();
            if (left || right) {
                if (left) {
                    listWidget.index--;
                    if (listWidget.index < 0)
                        listWidget.index = 0;
                }
                if (right) {
                    listWidget.index++;
                    if (listWidget.index >= listWidget.list.length)
                        listWidget.index = listWidget.list.length - 1;
                }
                listWidget.callback.accept(listWidget.index, listWidget.list[listWidget.index]);
            }
        } else if (widget instanceof ActivationWidget) {
            if (accept()) {
                ActivationWidget activationWidget = (ActivationWidget) widget;
                activationWidget.callback.apply(true);
            }
        } else if (widget instanceof StatsWidget) {
            if (accept()) {
                menuStack.add(widget);
            }
        } else if (widget instanceof MenuWidget) {
            if (accept()) {
                menuStack.add(widget);
            }
        } else if (widget instanceof RunWidget) {
            RunWidget runWidget = (RunWidget) widget;
            if (accept()) {
                if ((runWidget.isEnabled == null) || (runWidget.isEnabled.getAsBoolean())) {
                    runWidget.runnable.run();
                }
            }
        }

        return output.toString();
    }

    // Add a new widget to the appropriate spot in the menu hierarchy:
    private void add(String descriptor, Widget newWidget) {
        MenuWidget menu = (MenuWidget) menuStack.get(0); // Root menu

        // Peel off the hierarchy which is in the form "Vision::Configuration::Setting":
        while (true) {
            int index = descriptor.indexOf(DESCRIPTOR_SEPARATOR);
            if (index == -1)
                break; // ====>

            // Peel off the first menu name from the descriptor:
            String submenuName = descriptor.substring(0, index);
            descriptor = descriptor.substring(index + DESCRIPTOR_SEPARATOR.length());

            // Find or create the submenu:
            MenuWidget submenu = null;
            for (Widget widget : menu.widgets) {
                if ((widget instanceof MenuWidget) && (widget.description.equals(submenuName))) {
                    submenu = (MenuWidget) widget;
                    break;
                }
            }
            if (submenu == null) {
                submenu = new MenuWidget(submenuName);
                menu.widgets.add(submenu);
            }
            // Descend into that submenu:
            menu = submenu;
        }
        menu.widgets.add(newWidget);
    }

    // Add a toggleable widget to the menu:
    /** @noinspection unused*/
    public static void addToggle(String descriptor, boolean initialValue, Consumer<Boolean> callback) {
        callback.accept(initialValue);
        gui.add(descriptor, new ToggleWidget(descriptor, initialValue, callback));
    }
    // Add a list widget to the menu:
    /** @noinspection unused*/
    public static void addList(String descriptor, String[] list, int initialIndex, BiConsumer<Integer, String> callback) {
        callback.accept(initialIndex, list[initialIndex]);
        gui.add(descriptor, new ListWidget(descriptor, initialIndex, list, callback));
    }
    // Add a widget that can only be activated:
    /** @noinspection unused*/
    public static void addActivation(String descriptor, Function<Boolean, String> callback) {
        callback.apply(true);
        gui.add(descriptor, new ActivationWidget(descriptor, callback));
    }
    /** @noinspection unused*/
    public static void addStats(String descriptor, Supplier<String> callback) {
        gui.add(descriptor, new StatsWidget(descriptor, callback));
    }
    // Add a widget that can be run:
    public static void addRunnable(String descriptor, Runnable callback, BooleanSupplier isEnabled) {
        gui.add(descriptor, new RunWidget(descriptor, callback, isEnabled));
    }
    public static void addRunnable(String descriptor, Runnable callback) {
        gui.add(descriptor, new RunWidget(descriptor, callback, ()->true));
    }
}

/**
 * Looney Tuner's opMode class for running the tuning tests.
 *
 * @noinspection UnnecessaryUnicodeEscape, AccessStaticViaInstance, ClassEscapesDefinedScope
 */
@SuppressLint("DefaultLocale")
@TeleOp
public class LooneyTuner extends LinearOpMode {
    static final String A = "\ud83c\udd50"; // Symbol for the gamepad A button
    static final String B = "\ud83c\udd51"; // Symbol for the gamepad B button
    static final String X = "\ud83c\udd67"; // Symbol for the gamepad X button
    static final String Y = "\ud83c\udd68"; // Symbol for the gamepad Y button

    // Member fields referenced by every test:
    Gui gui;
    Dialogs dialogs;
    MecanumDrive drive;
    TuneParameters currentParameters;
    TuneParameters originalParameters;

    // Constants:
    final Pose2d zeroPose = new Pose2d(0, 0, 0);

    // Check if the robot code setting the MecanumDrive configuration parameters is up to date
    // with the last results from tuning:
    /** @noinspection BusyWait*/
    static public void verifyCodeMatchesTuneResults(MecanumDrive drive, Telemetry telemetry, Gamepad gamepad) {
        // There's no point in complaining about mismatches when running under the simulator:
        if (WilyWorks.isSimulating)
            return; // ====>

        TuneParameters currentSettings = new TuneParameters(drive);
        TuneParameters savedSettings = currentSettings.getSavedParameters();
        if (savedSettings != null) {
            String comparison = savedSettings.compare(currentSettings, false);
            if (!comparison.isEmpty()) {
                // There is no way to query the current display format so we have to assume it
                // could be either HTML or non-HTML.
                telemetry.clear();
                telemetry.addLine("YOUR CODE IS OUT OF SYNC WITH LOONEY TUNER");
                telemetry.addLine();
                telemetry.addLine("The code's configuration parameters don't match the last "
                        + "results saved in Looney Tuner. To use the Looney Tuner results, double-tap the shift key in Android "
                        + "Studio, enter 'md.params' to jump to the MecanumDrive Params constructor, "
                        + "then update as follows:");
                telemetry.addLine();
                telemetry.addLine(comparison);
                telemetry.addLine("Please update your code and restart now. Or, to proceed anyway and "
                        + "delete the Looney Tuner results, triple-tap the BACK button on the gamepad.");
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

    /**
     * Class that encapsulates the dialogs framework.
     */
    class Dialogs {
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
            while (opModeIsActive() && !gui.cancel()) {
                message(message);
                if (gui.accept()) {
                    success = true;
                    break;
                }
                updateGamepadDriving();
                updateRotation();
            }
            stopMotors();
            drive.setPose(zeroPose); // Reset the pose once they stopped
            return success;
        }

        // Show a message and wait for an A or B button press. If accept (A) is pressed, return
        // success. if cancel (B) is pressed, return failure. The robot CANNOT be driven while
        // waiting.
        boolean staticPrompt(String message) {
            while (opModeIsActive() && !gui.cancel()) {
                message(message);
                if (gui.accept())
                    return true;
            }
            return false;
        }
    }

    // Return a string that represents the distance the test will run:
    String testDistance(int distance) {
        return String.format("%d inches (%.1f tiles)", distance, distance / 24.0);
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
        while (opModeIsActive() && !gui.cancel()) {
            TelemetryPacket packet = MecanumDrive.getTelemetryPacket();
            dialogs.message("Press "+B+" to stop");
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
        if (Math.abs(result) > Math.abs(stickValue))
            out.printf("LooneyTuner: raw stick: %.2f, shaped: %.2f, power: %.2f, signum: %.2f\n", stickValue, result, power, Math.signum(stickValue));
        return result;
    }

    // Poll the gamepad input and set the drive motor power accordingly:
    public void updateGamepadDriving() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(
            shapeStick(-gamepad1.left_stick_y),
            shapeStick(-gamepad1.left_stick_x)),
            shapeStick(-gamepad1.right_stick_x)));
    }

    // Measure the optical linear scale and orientation:
    void pushTuner() {
        final int DISTANCE = 96; // Test distance in inches
        useDrive(false); // Don't use MecanumDrive/TankDrive

        // Reset the current OTOS settings:
        Pose2D oldOffset = drive.PARAMS.otos.offset;
        double oldLinearScalar = drive.PARAMS.otos.linearScalar;
        drive.opticalTracker.setOffset(new Pose2D(oldOffset.x, oldOffset.y, 0));
        drive.opticalTracker.setLinearScalar(1.0);

        if (dialogs.drivePrompt("In this test, you'll push the robot forward in a straight line "
                + "along a field wall for exactly "+testDistance(DISTANCE)+". To start, align the robot by hand "
                + "at its starting point along a wall. "
                + "\n\nPress "+A+" when in position, "+B+" to cancel.")) {

            double distance = 0;
            double heading = 0;

            drive.opticalTracker.resetTracking();

            boolean success = false;
            while (opModeIsActive() && !gui.cancel()) {
                if (gui.accept()) {
                    success = true;
                    break; // ====>
                }

                Pose2D pose = drive.opticalTracker.getPosition();
                distance = Math.hypot(pose.x, pose.y);
                heading = -Math.atan2(pose.y, pose.x); // Rise over run

                dialogs.message("Push forward exactly "+testDistance(DISTANCE)+" along the field wall.\n\n"
                    + String.format("&ensp;Sensor reading: (%.1f\", %.1f\", %.1f\u00b0)\n", pose.x, pose.y, Math.toDegrees(pose.h))
                    + String.format("&ensp;Effective distance: %.2f\"\n", distance)
                    + String.format("&ensp;Heading angle: %.2f\u00b0\n", Math.toDegrees(heading))
                    + "\nPress "+A+" when you're finished pushing, "+B+" to cancel");
            }

            if (success) {
                // Avoid divide-by-zeroes on aborts:
                if (distance == 0)
                    distance = 0.001;

                TuneParameters newParameters = currentParameters.createClone();
                newParameters.params.otos.linearScalar = (DISTANCE / distance);
                newParameters.params.otos.offset.h = normalizeAngle(heading);

                double linearScalarChange = Math.abs((oldLinearScalar - newParameters.params.otos.linearScalar)
                        / oldLinearScalar * 100.0); // Percentage
                double headingChange = normalizeAngle(Math.abs(oldOffset.h - newParameters.params.otos.offset.h));

                if (newParameters.params.otos.linearScalar < SparkFunOTOS.MIN_SCALAR) {
                    String message = String.format("The measured distance of %.1f\" is not close enough to "
                            + "the expected distance of %d\". It can't measure more than %.1f\". "
                            + "Either you didn't push straight for "+testDistance(DISTANCE)+" or something is wrong "
                            + "with the sensor. ", distance, DISTANCE, DISTANCE / SparkFunOTOS.MIN_SCALAR);
                    message += "Maybe the distance of the sensor to the tile is less than 10.0 mm? ";
                    dialogs.staticPrompt(message + "\n\nAborted, press "+A+" to continue");
                } else if (newParameters.params.otos.linearScalar > SparkFunOTOS.MAX_SCALAR) {
                    String message = String.format("The measured distance of %.1f\" is not close enough to "
                            + "the expected distance of %d\". It can't measure less than %.1f\". "
                            + "Either you didn't push straight for "+testDistance(DISTANCE)+" or something is wrong "
                            + "with the sensor. ", distance, DISTANCE, DISTANCE / SparkFunOTOS.MAX_SCALAR);

                    // If the measured distance is close to zero, don't bother with the following
                    // suggestion:
                    if (newParameters.params.otos.linearScalar < 1.5) {
                        message += "Maybe the distance of the sensor to the tile is more than 10.0 mm?";
                    }
                    dialogs.staticPrompt(message + "\n\nAborted, press "+A+" to continue");
                } else {
                    if (dialogs.staticPrompt(String.format("New offset heading %.3f\u00b0 is %.1f\u00b0 off from old.\n",
                            Math.toDegrees(newParameters.params.otos.offset.h), Math.toDegrees(headingChange))
                            + String.format("New linear scalar %.3f is %.1f%% off from old.\n\n",
                            newParameters.params.otos.linearScalar, linearScalarChange)
                            + "Use these results? Press "+A+" if they look good, "+B+" to cancel.")) {

                        acceptParameters(newParameters);
                    }
                }
            }
        }

        // Set the hardware to the new (or old) settings:
        setOtosHardware();
    }

    // Prompt the user for how to set the new parameters and save them to the registry:
    public void acceptParameters(TuneParameters newParameters) {
        String comparison = newParameters.compare(currentParameters, true);
        if (comparison.isEmpty()) {
            dialogs.staticPrompt("The new results match your current settings.\n\nPress "+A+" to continue.");
        } else {
            MecanumDrive.PARAMS = newParameters.params;
            currentParameters = newParameters;
            currentParameters.save();
            dialogs.staticPrompt("Double-tap the shift key in Android Studio, enter '<b>md.params</b>' to jump to the "
                    + "MecanumDrive Params constructor, then update as follows:\n\n"
                    + comparison
                    + "\nPress "+A+" to continue.");
        }
    }

    // Show all of the parameters that have been updated in this run:
    public void showUpdatedParameters() {
        String comparison = currentParameters.compare(originalParameters, true);
        if (comparison.isEmpty()) {
            dialogs.staticPrompt("There are no changes from your current settings.\n\nPress "+A+" to continue.");
        } else {
            dialogs.staticPrompt("Here are all of the parameter updates from your current run. "
                    + "Double-tap the shift key in Android Studio, enter 'md.params' to jump to the "
                    + "MecanumDrive Params constructor, then update as follows:\n\n"
                    + comparison
                    + "\nPress "+A+" to continue.");
        }
    }

    // Set the hardware to the current parameters:
    public void setOtosHardware() {
        drive.initializeOpticalTracker();
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
        accumulatedSparkFunRotation = 0;
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

    // Draw the spin sample points, plus the optional best-fit circle, on FTC Dashboard:
    void drawSpinPoints(ArrayList<Point> points, Circle circle) {
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

        final double REVOLUTION_COUNT = 10.0; // Number of revolutions to use
        final double SPIN_POWER = 0.5; // Speed of the revolutions

        useDrive(true); // Use MecanumDrive/TankDrive

        // Zero these settings for the purpose of this test:
        drive.opticalTracker.setOffset(new Pose2D(0, 0, 0));
        drive.opticalTracker.setAngularScalar(0);

        if (dialogs.drivePrompt("In this test, you'll position the robot against a wall, then drive "
                + String.format("it out so that the robot can rotate in place %.1f times, then position ", REVOLUTION_COUNT)
                + "the robot against the wall again."
                + "\n\nFirst, carefully drive the robot to a wall and align it so that "
                + "it's facing forward. This marks the start orientation for calibration."
                + "\n\nDrive the robot to the start position, press "+A+" when ready, "+B+" to cancel")) {

            // Let the user position the robot:
            if (dialogs.drivePrompt("Now move the robot far enough away from the wall (and any objects) so "
                    + "that it can freely rotate in place."
                    + "\n\nPress "+A+" when ready for the robot to rotate, "+B+" to cancel")) {

                ArrayList<Point> points = new ArrayList<>();

                // Spin-up the robot, starting to measure rotation for the 'scalar' computation at
                // this point:
                double scalarStartRotation = initiateSparkFunRotation();
                rampMotorsSpin(drive, SPIN_POWER);

                // Prepare for calculating how far the wheels have traveled:
                double voltageSum = 0;
                int voltageSamples = 0;
                double startTime = time();

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
                do {
                    // Sample the voltage to compute an average later:
                    voltageSum += drive.voltageSensor.getVoltage();
                    voltageSamples++;

                    // Check if we're at the start of a new circle:
                    double offsetRotation = getSparkFunRotation() - offsetStartRotation;
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

                    // Update the telemetry:
                    drawSpinPoints(points, null);
                    double rotationsRemaining = (terminationRotation - offsetRotation) / (2 * Math.PI);
                    telemetryAdd(String.format("%.2f rotations remaining, %d samples", rotationsRemaining, points.size()));
                    telemetryAdd("\nPress "+B+" to abort.");
                    telemetryUpdate();

                    // Update for next iteration of the loop:
                    Pose2D rawPose = updateRotationAndGetPose();
                    rawPosition = new Point(rawPose.x, rawPose.y);
                } while (opModeIsActive() && !gui.cancel());

                double endTime = time();

                // Stop the rotation:
                rampMotorsSpin(drive, 0);

                if ((success) && dialogs.drivePrompt("Now drive the robot to align it at the wall in the same "
                        + "place and orientation as it started."
                        + "\n\nDrive the robot to its wall position, press "+A+" when done, "+B+" to cancel")) {

                    Circle circle = fitCircle(points, farthestPoint.x / 2, farthestPoint.y / 2);

                    // Draw results with the fitted circle:
                    drawSpinPoints(points, circle);

                    updateRotationAndGetPose();
                    double averageVoltage = voltageSum / voltageSamples;
                    double averageVelocity = (SPIN_POWER * averageVoltage - drive.PARAMS.kS) /
                            (drive.PARAMS.kV / drive.PARAMS.inPerTick); // Velocity in inches per second

                    double totalMeasuredRotation = getSparkFunRotation() - scalarStartRotation;
                    double distancePerRevolution = averageVelocity * (endTime - startTime) / REVOLUTION_COUNT;
                    processSpinResults(circle, totalMeasuredRotation, distancePerRevolution);
                }
            }
        }

        // Restore the hardware settings:
        setOtosHardware();
    }

    // Process the spin results:
    void processSpinResults(Circle center, double totalMeasuredRotation, double distancePerRevolution) {
        double totalMeasuredCircles = totalMeasuredRotation / (2 * Math.PI);
        double integerCircles = Math.round(totalMeasuredCircles);
        double angularScalar = integerCircles / totalMeasuredCircles;

        // Now that we have measured the angular scalar, we can correct the distance-per-revolution:
        distancePerRevolution *= angularScalar;

        // 'Track width' is really the radius of the circle needed to make a complete rotation:
        double trackWidth = distancePerRevolution / (2 * Math.PI);
        double trackWidthTicks = trackWidth / drive.PARAMS.inPerTick;

        // Undo the offset heading that the OTOS sensor automatically applies:
        Point rawOffset = new Point(center.x, center.y).rotate(-drive.PARAMS.otos.offset.h);

        // Our initial origin where we established (0, 0) at the start of every circle is much
        // less reliable than the best-fit circle center and radius because the former uses
        // only a single sample point while the latter uses all sample points. Consequently, make
        // the offset fit the radius while maintaining the same angle to the center of the circle:
        double theta = rawOffset.atan2();
        Point offset = new Point(Math.cos(theta) * center.radius, Math.sin(theta) * center.radius);

        String results = String.format("Sensor thinks %.2f circles were completed.\n\n", totalMeasuredCircles);
        results += String.format("Circle-fit position: (%.2f, %.2f), radius: %.2f\n", rawOffset.x, rawOffset.y, center.radius);
        results += String.format("Radius-corrected position: (%.2f, %.2f)\n", offset.x, offset.y);
        results += String.format("Angular scalar: %.3f\n", angularScalar);
        results += String.format("Track width: %.2f\"\n", trackWidth);
        results += "\n";

        // Do some sanity checking on the results:
        if ((Math.abs(offset.x) > 12) || (Math.abs(offset.y) > 12)) {
            dialogs.staticPrompt(results + "The results are bad, the calculated center-of-rotation is bogus.\n\n"
                    + "Aborted, press "+A+" to continue.");
        } else if  ((angularScalar < SparkFunOTOS.MIN_SCALAR) || (angularScalar > SparkFunOTOS.MAX_SCALAR)) {
            dialogs.staticPrompt(results + "The measured number of circles is bad. Did you properly align "
                    + "the robot on the wall the same way at both the start and end of this test?\n\n"
                    + "Aborted, press "+A+" to continue.");
        } else {
            TuneParameters newSettings = currentParameters.createClone();
            newSettings.params.otos.offset.x = offset.x;
            newSettings.params.otos.offset.y = offset.y;
            newSettings.params.otos.angularScalar = angularScalar;
            newSettings.params.trackWidthTicks = trackWidthTicks;

            if (dialogs.staticPrompt(results + "Use these results? Press "+A+" if they look good, "+B+" to discard them.")) {
                acceptParameters(newSettings);

                // We changed 'trackWidthTicks' so recreate the kinematics object:
                drive.recreateKinematics();
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
        final int DISTANCE = 72; // Test distance in inches

        final double VELOCITY_EPSILON = 2.0; // Inches/s
        final double POWER_FACTOR_ADDER_PER_SECOND = 0.1;
        final double MIN_POWER_FACTOR = 0.05;
        final double MAX_POWER_FACTOR = 0.9;

        useDrive(true); // Set the brakes
        assert(drive.opticalTracker != null);

        if (dialogs.drivePrompt("The robot will drive forward for up to " + testDistance(DISTANCE) + ". "
                + "It will start slowly but get faster and faster. "
                + "\n\nDrive the robot to a good spot, press "+A+" to start, "+B+" to cancel.")) {

            ArrayList<Point> points = new ArrayList<>();
            double startTime = time();
            double maxVelocity = 0; // Inches/s
            double maxPower = 0; // Volts
            boolean success = false;
            while (opModeIsActive()) {
                // Slowly ramp up the voltage. Increase power by the specified power adder:
                double scaledPower = (time() - startTime) * POWER_FACTOR_ADDER_PER_SECOND;
                double powerFactor = scaledPower + MIN_POWER_FACTOR;

                drive.rightFront.setPower(powerFactor);
                drive.rightBack.setPower(powerFactor);
                drive.leftFront.setPower(powerFactor);
                drive.leftBack.setPower(powerFactor);

                Pose2D velocityVector = drive.opticalTracker.getVelocity();
                double velocity = Math.hypot(velocityVector.x, velocityVector.y);

                // Discard zero velocities that will happen when the provided power isn't
                // enough yet to overcome static friction:
                if (velocity > VELOCITY_EPSILON) {
                    double power = powerFactor * drive.voltageSensor.getVoltage();
                    maxPower = Math.max(power, maxPower);
                    points.add(new Point(velocity, power));
                    maxVelocity = Math.max(velocity, maxVelocity);
                }
                // We're done if we've reach the maximum target voltage:
                if (powerFactor > MAX_POWER_FACTOR) {
                    success = true;
                    break;
                }
                // We're also done if we've gone far enough:
                Pose2D position = drive.opticalTracker.getPosition();
                double distance = Math.hypot(position.x, position.y);
                if (distance > DISTANCE) {
                    success = true;
                    break;
                }

                double remaining = Math.max(0, DISTANCE - distance);
                telemetryAdd(String.format("Inches remaining: %.1f, power: %.1f\n", remaining, powerFactor));
                telemetryAdd("Press "+B+" to abort.");
                telemetryUpdate();
            }

            // Stop the robot:
            stopMotors();
            if (success) {
                if (maxVelocity == 0) {
                    dialogs.staticPrompt("The optical tracking sensor returned only zero velocities. "
                            + "Is it working properly?"
                            + "\n\nAborted, press "+A+" to continue.");
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
                    double yScale = 140 / maxPower;

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

                    TuneParameters newParameters = currentParameters.createClone();
                    newParameters.params.kS = bestFitLine.intercept;
                    newParameters.params.kV = bestFitLine.slope * currentParameters.params.inPerTick;

                    if (dialogs.staticPrompt("Check out the graph on FTC Dashboard!\n\n"
                            + String.format("&ensp;New kS: %.03f, old kS: %.03f\n", newParameters.params.kS, currentParameters.params.kS)
                            + String.format("&ensp;New kV: %.06f, old kV: %.06f\n", newParameters.params.kV, currentParameters.params.kV)
                            + "\nIf these look good, press " + A + " to accept, " + B + " to cancel.")) {

                        acceptParameters(newParameters);
                    }
                }
            }
        }

        setOtosHardware();
    }

    // Test the robot motors.
    void wheelDebugger() {
        String[] motorDescriptions = { "leftFront", "leftBack", "rightBack", "rightFront" };
        DcMotorEx[] motors = { drive.leftFront, drive.leftBack, drive.rightBack, drive.rightFront };
        int i = 0;

        stopMotors();
        while (opModeIsActive() && !gui.cancel()) {
            double power = gamepad1.right_trigger - gamepad1.left_trigger;
            String description = motorDescriptions[i];
            telemetryAdd(String.format("This tests every motor individually, now testing '%s'.\n\n", description)
                + String.format("&emsp;%s.setPower(%.2f)\n\n", description, power)
                + "Press right trigger for forward, left trigger for reverse, "+X+" for next motor, "+B+" to cancel.");
            telemetryUpdate();

            motors[i].setPower(power);
            if (gui.xButton()) {
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

        while (opModeIsActive() && !gui.cancel()) {
            if (gui.xButton())
                wheelDebugger();
            if (gui.yButton())
                drive.setPose(zeroPose);

            updateGamepadDriving();
            drive.updatePoseEstimate();

            TelemetryPacket p = MecanumDrive.getTelemetryPacket();
            Pose2d pose = drive.pose;
            dialogs.message("Use the controller to drive the robot around.\n\n"
                    + String.format("&ensp;Pose: (%.2f\", %.2f\", %.2f\u00b0)\n", pose.position.x, pose.position.y, pose.heading.toDouble())
                    + "\nPress "+X+" to debug motor wheels, "+Y+" to reset the pose, "+B+" to exit.");

            Canvas c = p.fieldOverlay();
            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, drive.pose);
            MecanumDrive.sendTelemetryPacket(p);

        }
        stopMotors();
    }

    // Tuner for the lateral multiplier on Mecanum drives.
    void lateralTuner() {
        final int DISTANCE = 48; // Test distance in inches

        useDrive(true); // Do use MecanumDrive/TankDrive

        if (dialogs.drivePrompt("The robot will strafe left for " + testDistance(DISTANCE) + ". "
                + "Please ensure that there is sufficient room."
                + "\n\nDrive the robot to position, press "+A+" to start, "+B+" to cancel")) {

            // Disable the PID gains so that the distance traveled isn't corrected:
            TuneParameters testParameters = currentParameters.createClone();
            testParameters.params.lateralGain = 0;
            testParameters.params.lateralVelGain = 0;
            testParameters.params.lateralInPerTick = 1.0;

            // Point the MecanumDrive to our test PARAMS:
            MecanumDrive.PARAMS = testParameters.params;

            // Now recreate the Kinematics object based on the new settings:
            drive.recreateKinematics();

            // Drive at a slow speed:
            double maxVelocity = drive.PARAMS.maxWheelVel / 4;

            drive.opticalTracker.setPosition(new Pose2D(0, 0, 0));
            if (runCancelableAction(drive.actionBuilder(drive.pose)
                            .strafeTo(new Vector2d(0, DISTANCE), new TranslationalVelConstraint(maxVelocity))
                            .build())) {

                Pose2D endPose = drive.opticalTracker.getPosition();
                double actualDistance = Math.hypot(endPose.x, endPose.y);
                double lateralMultiplier = actualDistance / DISTANCE;
                double inchesPerTick = currentParameters.params.inPerTick * lateralMultiplier;

                if (lateralMultiplier < 0.25) {
                    dialogs.staticPrompt("The measured distance is too low to be correct. "
                            + "Did it not move, or is the distance sensor not working properly?"
                            + "\n\nPress "+A+"to continue.");
                } else if (dialogs.staticPrompt(String.format("Measured lateral multiplier is %.3f. ", lateralMultiplier)
                        + "Does that look good?"
                        + "\n\nPress "+A+" to accept, "+B+" to cancel")) {
                    TuneParameters newParameters = currentParameters.createClone();
                    newParameters.params.lateralInPerTick = inchesPerTick;
                    acceptParameters(newParameters);
                }
            }

            // We're done, set to our new state or back to the original state:
            MecanumDrive.PARAMS = currentParameters.params;
            drive.recreateKinematics();
        }
    }

    // Tune the kV and kA feed forward parameters:
    void interactiveFeedForwardTuner() {
        final int DISTANCE = 72; // Test distance in inches
        useDrive(false); // Don't use MecanumDrive/TankDrive

        // Disable all lateral gains so that backward and forward behavior is not affected by the
        // PID/Ramsete algorithm. It's okay for the axial and rotation gains to be either zero
        // or non-zero:
        TuneParameters testParameters = currentParameters.createClone();
        testParameters.params.lateralGain = 0;
        testParameters.params.lateralVelGain = 0;
        MecanumDrive.PARAMS = testParameters.params;

        int inputIndex = 0;
        NumericInput[] numericInputs = {
            new NumericInput(drive.PARAMS, "kV", -2, 6, 0.000001, 20),
            new NumericInput(drive.PARAMS, "kA", -3, 5, 0, 1),
        };

        // Register the variables with non-zero results now so that they can be registered for
        // graphing in FTC Dashboard even before the first test is run:
        TelemetryPacket packet = MecanumDrive.getTelemetryPacket();
        packet.put("\u23af\u23af\u23af\u23af\u23af\u23af\u23af\u23af\u23af\u23af\u23af\u23af\u23af", "\u23af");
        packet.put("vRef", 0.001);
        packet.put("vActual", 0.001);
        MecanumDrive.sendTelemetryPacket(packet);

        if (dialogs.drivePrompt("The robot will drive forwards then backwards for " + testDistance(DISTANCE) + ". "
                + "Tune 'kV' and 'kA' using FTC Dashboard. Follow "
                + "<u><a href='https://learnroadrunner.com/feedforward-tuning.html#tuning'>LearnRoadRunner's guide</a></u>.\n\n"
                + "Press "+A+" to start, "+B+" to cancel")) {

            // Trigger a reset the first time into the loop:
            double startTime = 0;
            double maxVelocityFactor = 1.0;
            TimeProfile profile = null;
            boolean movingForwards = false;
            int queuedXButtons = 0;

            while (opModeIsActive()) {
                // Process the gamepad numeric input:
                numericInputs[inputIndex].update();

                if (inputIndex == 0) {
                    telemetryAdd("Graph <b>vActual</b> and <b>vRef</b> using FTC Dashboard. "
                            + "Adjust kV to make the horizontal lines as close as possible in height. "
                            + "Remember, <i>kV = vRef / vActual</i>.\n\n"
                            + "If there are no horizontal lines, decrease the maximum velocity using the left trigger.\n");
                } else {
                    telemetryAdd("Graph <b>vActual</b> and <b>vRef</b> using FTC Dashboard. "
                            + "Adjust <b>kA</b> to shift <b>vActual</b> left and right so the angled lines overlap.\n");
                }

                if (gui.xButton())
                    queuedXButtons++; // Let the x-button be queued up even while running

                // If there's a profile, that means we're actively moving:
                if (profile != null) {
                    telemetryAdd("Press " + B + " to cancel");
                    telemetryUpdate();

                    Pose2D velocityPose = drive.opticalTracker.getVelocity();
                    double velocity = Math.signum(velocityPose.x) * Math.hypot(velocityPose.x, velocityPose.y);
                    packet = MecanumDrive.getTelemetryPacket();
                    packet.put("vActual", velocity);

                    double t = time() - startTime;
                    if (t > profile.duration) {
                        if (movingForwards) {
                            movingForwards = false;
                            startTime = time();
                        } else {
                            profile = null;
                            continue; // ====>
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

                    if (gui.cancel()) {
                        // Cancel the current cycle but remain in this test:
                        stopMotors();
                        queuedXButtons = 0;
                        profile = null;
                    }
                } else {
                    telemetryAdd(String.format("Max velocity set to <b>%.0f%%</b>. Change using triggers.\n",
                            maxVelocityFactor * 100.0));

                    telemetryAdd("Press "+X+" to run, "+A+" when done, "+B+" to cancel, "
                            +Y+" to switch gain variables");
                    telemetryUpdate();

                    updateGamepadDriving();

                    if (gui.accept()) {
                        // Make sure that the user does both kV and kA:
                        if (inputIndex == 0)
                            inputIndex = 1;
                        else {
                            if (dialogs.staticPrompt("Happy with your results?\n\nPress "+A+" to accept, "+B+" to cancel")) {

                                // Reset the state that we zeroed to run the test:
                                testParameters.params.lateralGain = currentParameters.params.lateralGain;
                                testParameters.params.lateralVelGain = currentParameters.params.lateralVelGain;
                                acceptParameters(testParameters);
                            }
                            break; // ====>
                        }
                    }
                    if (gui.cancel()) {
                        if (dialogs.staticPrompt("Are you sure you want to discard your current input?\n\n"
                                + "Press "+A+" to discard, "+B+" to cancel."))
                            break; // ====>
                    }
                    if (gui.leftTrigger())
                        maxVelocityFactor = Math.max(maxVelocityFactor - 0.1, 0.2);
                    if (gui.rightTrigger())
                        maxVelocityFactor = Math.min(maxVelocityFactor + 0.1, 1.0);
                    if (gui.yButton())
                        inputIndex ^= 1;
                    if (queuedXButtons > 0) {
                        queuedXButtons--;
                        stopMotors(); // Stop the user's driving
                        movingForwards = true;
                        startTime = time();
                        profile = new TimeProfile(constantProfile(
                                DISTANCE, 0.0,
                                MecanumDrive.PARAMS.maxWheelVel * maxVelocityFactor,
                                MecanumDrive.PARAMS.minProfileAccel,
                                MecanumDrive.PARAMS.maxProfileAccel).baseProfile);

                        // Reset the start position on every cycle. This ensures that getVelocity().x
                        // is the appropriate velocity to read, and it resets after we reposition
                        // the robot:
                        drive.setPose(new Pose2d(-DISTANCE / 2.0, 0, 0));
                    }
                }
            }
        }

        // We're done, undo any temporary state we set:
        MecanumDrive.PARAMS = currentParameters.params;
        stopMotors();
    }

    /**
     * Class to handle gamepad input of decimal numbers.
     */
    class NumericInput {
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
        int lastInput; // Last quantized input (-1, 0 or 1)
        double nextAdvanceTime; // Time at which to advance the value

        // Take a reference to the object and the name of its field to be updated. We use reflection
        // to make the calling code's life a little easier. 'startDigit' dictates the starting
        // focus, 'decimalDigits' is the number of decimal digits to support, 'minValue' and
        // 'maxValue' specify the acceptable ranges.
        NumericInput(Object object, String fieldName, int startDigit, int decimalDigits, double minValue, double maxValue) {
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
        void update() {
            telemetryAdd(String.format("Here's the current value for <b>%s</b>. ", fieldName)
                + "Press Dpad up/down to change its value, right/left to move the cursor.\n");

            if (gui.left()) {
                digit = Math.min(digit + 1, 2);
            }
            if (gui.right()) {
                digit = Math.max(digit - 1, -decimalDigits);
            }
            if ((digit > 0) && (Math.pow(10, digit) > Math.abs(value))) {
                digit--;
            }

            // Advance the value according to the thumb stick state:
            int input = gui.gamepad.dpad_up ? 1 : (gui.gamepad.dpad_down ? -1 : 0); // -1, 0 or 1
            if (input != lastInput) {
                nextAdvanceTime = time() + INITIAL_DELAY;
                lastInput = input;
                value += lastInput * Math.pow(10, digit);
            } else if (time() > nextAdvanceTime) {
                nextAdvanceTime = time() + ADVANCE_DELAY;
                value += lastInput * Math.pow(10, digit);
            }

            // Clamp new value to acceptable range:
            value = Math.max(minValue, Math.min(maxValue, value));

            // Set the value into the class:
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

            telemetryAdd(String.format("<big><big>&emsp;%s%s%s</big></big>\n", prefix, middle, suffix));
        }
    }

    // Types of interactive PiD tuners:
    enum PidTunerType { AXIAL, LATERAL, HEADING }

    // Adjust the Ramsete/PID values:
    void interactivePidTuner(PidTunerType type) {
        final int DISTANCE = 48; // Test distance in inches
        useDrive(true); // Do use MecanumDrive/TankDrive

        TuneParameters testParameters = currentParameters.createClone();
        MecanumDrive.PARAMS = testParameters.params;
        String prompt, gainName, velGainName;

        TrajectoryActionBuilder trajectory = drive.actionBuilder(zeroPose);
        if (type == PidTunerType.AXIAL) {
            prompt = "The robot will drive forwards and then backwards " + testDistance(DISTANCE) + ". ";
            trajectory = trajectory.lineToX(DISTANCE).lineToX(0);
            gainName = "axialGain";
            velGainName = "axialVelGain";

        } else if (type == PidTunerType.LATERAL) {
            prompt = "The robot will strafe left and then right " + testDistance(DISTANCE) + ". ";
            trajectory = trajectory.strafeTo(new Vector2d(0, DISTANCE)).strafeTo(new Vector2d(0, 0));
            gainName = "lateralGain";
            velGainName = "lateralVelGain";

        } else {
            prompt = "The robot will rotate in place 180° clockwise and then counterclockwise. ";
            trajectory = trajectory.turn(Math.PI).turn(-Math.PI);
            gainName = "headingGain";
            velGainName = "headingVelGain";
        }
        if (dialogs.drivePrompt(prompt + "Tune the values to minimize error and make the target "
                + "and actual trajectories shown in FTC Dashboard align.\n\n"
                + "Press "+A+" to start, "+B+" to cancel")) {
            int inputIndex = 0;
            NumericInput[] numericInputs = {
                new NumericInput(drive.PARAMS, gainName, -1, 3, 0, 20),
                new NumericInput(drive.PARAMS, velGainName, -1, 3, 0, 20),
            };
            int queuedXButtons = 0;

            while (opModeIsActive()) {
                // Drive:
                TelemetryPacket packet = MecanumDrive.getTelemetryPacket();
                boolean more = drive.doActionsWork(packet);

                // Process the gamepad numeric input:
                numericInputs[inputIndex].update();

                // Compute the relevant error:
                double error;
                String errorString;
                if (type == PidTunerType.AXIAL) {
                    error = drive.pose.position.x - drive.targetPose.position.x;
                    errorString = String.format("%.2f\"", error);
                } else if (type == PidTunerType.LATERAL) {
                    error = drive.pose.position.y - drive.targetPose.position.y;
                    errorString = String.format("%.2f\"", error);
                } else {
                    error = Math.toDegrees(normalizeAngle(drive.pose.heading.toDouble()
                                                        - drive.targetPose.heading.toDouble()));
                    errorString = String.format("%.2f\u00b0", error);
                }

                String tuningHint = (inputIndex == 0)
                    ? "Tune to minimize measured error and get circles to align. "
                    : "Tune to minimize oscillations. ";

                telemetryAdd(tuningHint + "Press "+X+" to test this value.\n");

                packet.put("Error", error); // Make the error graphable

                if (gui.xButton())
                    queuedXButtons++; // Let the x-button be queued up even while running

                if (more) {
                    telemetryAdd("Current error: " + errorString + ".\n");
                    telemetryAdd("Press "+B+" to cancel");
                    telemetryUpdate();

                    // Only send the packet if there's more to the trajectory, otherwise the
                    // field view will be erased once the trajectory terminates:
                    MecanumDrive.sendTelemetryPacket(packet);

                    if (gui.cancel()) {
                        // Cancel the current cycle but remain in this test:
                        drive.abortActions();
                        queuedXButtons = 0;
                    }
                } else {
                    telemetryAdd("Last error: " + errorString + ".\n");
                    telemetryAdd("Press "+X+" to run, "+A+" when done, "+B
                            +" to cancel, "+Y+" to switch gain variables");
                    telemetryUpdate();

                    updateGamepadDriving();

                    if (gui.accept()) {
                        if (dialogs.staticPrompt("Happy with your results?\n\nPress "+A+" to accept, "+B+" to cancel")) {
                            acceptParameters(testParameters);
                            break; // ====>
                        }
                    }
                    if (gui.cancel()) {
                        if (dialogs.staticPrompt("Are you sure you want to discard your current input?\n\n"
                                + "Press "+A+" to discard, "+B+" to cancel."))
                            break; // ====>
                    }
                    if (gui.yButton())
                        inputIndex ^= 1; // Toggle the index

                    // If there is no more actions, let the X button start a new one.
                    if (queuedXButtons > 0) {
                        queuedXButtons--;
                        stopMotors(); // Stop the user's driving
                        drive.setPose(zeroPose);
                        if (type == PidTunerType.HEADING) {
                            // An apparent Road Runner build prevents a trajectory from being reused:
                            drive.runParallel(drive.actionBuilder(zeroPose).turn(Math.PI).turn(-Math.PI).build());
                        } else {
                            drive.runParallel(trajectory.build());
                        }
                    }
                }
            }
            drive.abortActions();
            stopMotors();

        }
        MecanumDrive.PARAMS = currentParameters.params;
    }

    // Navigate a short spline as a completion test.
    void completionTest() {
        useDrive(true); // Do use MecanumDrive/TankDrive

        if (dialogs.drivePrompt("The robot will drive forward 48 inches using a spline. "
                + "It needs half a tile clearance on either side. "
                + "\n\nDrive the robot to a good spot, press "+A+" to start, "+B+" to cancel")) {

            telemetryAdd("Press "+B+" to cancel");
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

        if (dialogs.drivePrompt("To test 'trackWidthTicks', the robot will turn in-place for two complete "
                + "rotations.\n\nPress "+A+" to start, "+B+" to cancel.")) {

            // Disable the rotational PID/Ramsete behavior so that we can test just the
            // feed-forward rotation:
            TuneParameters testParameters = currentParameters.createClone();
            testParameters.params.headingGain = 0;
            testParameters.params.headingVelGain = 0;
            MecanumDrive.PARAMS = testParameters.params;

            Action action = drive.actionBuilder(drive.pose)
                    .turn(2 * Math.toRadians(360), new TurnConstraints(
                            MecanumDrive.PARAMS.maxAngVel / 3,
                           -MecanumDrive.PARAMS.maxAngAccel,
                            MecanumDrive.PARAMS.maxAngAccel))
                    .build();
            runCancelableAction(action);

            dialogs.staticPrompt("The robot should be facing the same direction as when it started. It it's "
                + "not, run the spin tuner again to re-tune 'trackWidthTicks'."
                + "\n\nPress "+A+" to continue.");

            // Restore the parameters:
            MecanumDrive.PARAMS = currentParameters.params;
        }
    }

    @Override
    public void runOpMode() {
        // Set the display format to use HTML:
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        // Send telemetry to both FTC Dashboard and the Driver Station:
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize member fields:
        gui = new Gui(gamepad1);
        dialogs = new Dialogs();
        drive = new MecanumDrive(hardwareMap, telemetry, gamepad1, zeroPose);
        currentParameters = new TuneParameters(drive);
        originalParameters = currentParameters.createClone();

        if ((drive.opticalTracker != null) &&
            ((drive.opticalTracker.getAngularUnit() != AngleUnit.RADIANS) ||
             (drive.opticalTracker.getLinearUnit() != DistanceUnit.INCH))) {
            dialogs.staticPrompt("The SparkFun OTOS must be present and configured for radians and inches.");
            return; // ====>
        }

        // Dynamically build the list of tests:
        gui.addRunnable("Drive test (motors)", this::driveTest);
        if (drive.opticalTracker != null) {
            // Basic tuners:
            gui.addRunnable("Push tuner (OTOS orientation, linearScalar)", this::pushTuner);
            gui.addRunnable("Accelerating straight line tuner (kS and kV)", this::acceleratingStraightLineTuner,
                ()->drive.PARAMS.otos.linearScalar != 0);
            gui.addRunnable("Interactive feed forward tuner (kV and kA)", this::interactiveFeedForwardTuner,
                ()->drive.PARAMS.otos.linearScalar != 0 && drive.PARAMS.kS != 0 && drive.PARAMS.kV != 0);
            gui.addRunnable("Lateral tuner (lateralInPerTick)", this::lateralTuner,
                    ()->drive.PARAMS.otos.linearScalar != 0 && drive.PARAMS.kS != 0 && drive.PARAMS.kV != 0);
            gui.addRunnable("Spin tuner (OTOS angularScalar, offset)", this::spinTuner,
                    ()->drive.PARAMS.otos.linearScalar != 0 && drive.PARAMS.kS != 0 && drive.PARAMS.kV != 0);
            gui.addRunnable("Interactive PiD tuner (axialGain)", ()->interactivePidTuner(PidTunerType.AXIAL),
                ()->drive.PARAMS.otos.linearScalar != 0 && drive.PARAMS.kS != 0 && drive.PARAMS.kV != 0);
            gui.addRunnable("Interactive PiD tuner (lateralGain)", ()->interactivePidTuner(PidTunerType.LATERAL),
                ()->drive.PARAMS.otos.linearScalar != 0 && drive.PARAMS.lateralInPerTick != 0 && drive.PARAMS.kS != 0 && drive.PARAMS.kV != 0);
            gui.addRunnable("Interactive PiD tuner (headingGain)", ()->interactivePidTuner(PidTunerType.HEADING),
                ()->drive.PARAMS.otos.linearScalar != 0 && drive.PARAMS.trackWidthTicks != 0 && drive.PARAMS.kS != 0 && drive.PARAMS.kV != 0);
            gui.addRunnable("Completion test (overall verification)", this::completionTest,
                ()->drive.PARAMS.axialGain != 0 && drive.PARAMS.lateralGain != 0 && drive.PARAMS.headingGain != 0);

            // Extras:
            gui.addRunnable("Extras::Show accumulated parameter changes", this::showUpdatedParameters);
            gui.addRunnable("Extras::Rotation test (verify trackWidthTicks)", this::rotationTest,
                    ()->drive.PARAMS.trackWidthTicks != 0);
        }

        // Remind the user to press Start on the Driver Station, then press A on the gamepad.
        // We require the latter because enabling the gamepad on the DS after it's been booted
        // causes an A press to be sent to the app, and we don't want that to accidentally
        // invoke a menu option:
        dialogs.message("<big><big><big><big><big><big><big><big><b>Press \u25B6");
        waitForStart();
        dialogs.staticPrompt("<big><big><big><big><big><big><big><b>Press Gamepad "+A+" or "+B);

        while (opModeIsActive()) {
            telemetryAdd(gui.update());
            telemetryUpdate();
        }
    }
}