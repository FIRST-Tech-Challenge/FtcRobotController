package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.MainAuto;
import org.firstinspires.ftc.teamcode.systems.ShutdownManager;
import org.firstinspires.ftc.teamcode.systems.DynamicInput;
import org.firstinspires.ftc.teamcode.Settings;
import org.firstinspires.ftc.teamcode.utils.MenuHelper;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Base autonomous operation class that handles initial configuration and robot
 * setup.
 * Provides a menu system for selecting starting position and color, then
 * executes
 * the autonomous routine.
 */
@Autonomous(name = "Main Autonomous", group = "Autonomous")
public class AutoBase extends LinearOpMode {
    /** Selected alliance color (Red/Blue) */
    String color = "Red";

    /** Selected starting position (Left/Right) */
    String position = "Left";

    /** Menu options for autonomous configuration */
    private static final String[] MENU_OPTIONS = {
            "Red Left", "Red Right", "Blue Left", "Blue Right", "Confirm"
    };

    /**
     * Main autonomous execution flow:
     * 1. Displays configuration menu
     * 2. Initializes robot systems
     * 3. Executes autonomous routine
     * 4. Handles shutdown
     */
    @Override
    public void runOpMode() {
        // Initialize robot systems FIRST, before the menu
        DynamicInput dynamicInput = new DynamicInput(gamepad1, gamepad2,
                Settings.DEFAULT_PROFILE, Settings.DEFAULT_PROFILE);
        BaseRobot baseRobot = new BaseRobot(hardwareMap, dynamicInput, this, telemetry);

        // Add initialization status
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Give hardware time to initialize
        sleep(500); // Add a small delay for hardware initialization

        AtomicBoolean menuActive = new AtomicBoolean(true);
        AtomicInteger currentSelection = new AtomicInteger(0);

        // Menu loop
        while (!isStarted() && !isStopRequested() && menuActive.get()) {
            // Display menu header
            telemetry.addLine("=== Autonomous Configuration ===");
            telemetry.addLine("\nSelect Starting Position:");

            // Display options with cursor
            MenuHelper.displayMenuOptions(telemetry, MENU_OPTIONS, currentSelection.get());

            // Handle controller input with debounce
            MenuHelper.handleControllerInput(this, gamepad1, true, () -> {
                if (gamepad1.dpad_up) {
                    currentSelection.set((currentSelection.get() - 1 + MENU_OPTIONS.length) % MENU_OPTIONS.length);
                } else if (gamepad1.dpad_down) {
                    currentSelection.set((currentSelection.get() + 1) % MENU_OPTIONS.length);
                } else if (gamepad1.a) {
                    if (currentSelection.get() < MENU_OPTIONS.length - 1) {
                        String[] parts = MENU_OPTIONS[currentSelection.get()].split(" ");
                        color = parts[0]; // "Red" or "Blue"
                        position = parts[1]; // "Left" or "Right"
                    } else {
                        menuActive.set(false);
                    }
                }
            });

            // Display current selection
            telemetry.addLine("\nSelected Configuration:");
            telemetry.addData("Position",
                    (color != null && position != null) ? color + " " + position : "Not selected");
            telemetry.update();
        }

        // Create remaining components after menu selection
        MainAuto auto = new MainAuto(baseRobot, color, position);

        // Add ready status
        telemetry.addData("Status", "Ready to start!");
        telemetry.addData("Configuration", color + " " + position);
        telemetry.update();

        // Now wait for start
        waitForStart();

        try {
            if (opModeIsActive()) {
                auto.run(color + " " + position);
            }
        } catch (RuntimeException e) {
            // Shutdown handled by ShutdownManager
        }
    }
}