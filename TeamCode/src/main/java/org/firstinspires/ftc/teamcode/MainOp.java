package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Settings.ControllerProfile;
import org.firstinspires.ftc.teamcode.systems.DynamicInput;
import org.firstinspires.ftc.teamcode.utils.MenuHelper;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Main TeleOp class for driver-controlled period.
 * Handles controller profile selection and robot operation during matches.
 */
@TeleOp(name = "MainOp", group = "TeleOp")
public class MainOp extends LinearOpMode {

    /**
     * Main execution flow:
     * 1. Displays controller profile selection menu
     * 2. Initializes robot with selected profiles
     * 3. Runs main control loop for driver operation
     * 4. Handles shutdown when OpMode ends
     */
    @Override
    public void runOpMode() {
        // Show profile selection menu for both controllers
        AtomicReference<ControllerProfile> mainProfile = new AtomicReference<>(Settings.DEFAULT_PROFILE);
        AtomicReference<ControllerProfile> subProfile = new AtomicReference<>(Settings.DEFAULT_PROFILE);
        boolean menuActive = true;
        AtomicInteger mainSelection = new AtomicInteger();
        AtomicInteger subSelection = new AtomicInteger();
        AtomicBoolean mainConfirmed = new AtomicBoolean(false);
        AtomicBoolean subConfirmed = new AtomicBoolean(false);

        while (!isStarted() && !isStopRequested() && menuActive) {
            // Build options array
            String[] options = new String[Settings.AVAILABLE_PROFILES.length + 1];
            for (int i = 0; i < Settings.AVAILABLE_PROFILES.length; i++) {
                options[i] = Settings.AVAILABLE_PROFILES[i].name;
            }
            options[options.length - 1] = "Confirm";

            // Display menu header
            telemetry.addLine("=== Controller Profile Selection ===");

            // Main Controller Menu
            if (!mainConfirmed.get()) {
                telemetry.addLine("\nMain Controller (Gamepad 1):");
                MenuHelper.displayMenuOptions(telemetry, options, mainSelection.get());
            }

            // Sub Controller Menu
            if (!subConfirmed.get()) {
                telemetry.addLine("\nSub Controller (Gamepad 2):");
                MenuHelper.displayMenuOptions(telemetry, options, subSelection.get());
            }

            // Handle controller inputs with debounce
            MenuHelper.handleControllerInput(this, gamepad1, !mainConfirmed.get(), () -> {
                if (gamepad1.dpad_up) {
                    mainSelection.set((mainSelection.get() - 1 + options.length) % options.length);
                } else if (gamepad1.dpad_down) {
                    mainSelection.set((mainSelection.get() + 1) % options.length);
                } else if (gamepad1.a) {
                    if (mainSelection.get() < Settings.AVAILABLE_PROFILES.length) {
                        mainProfile.set(Settings.AVAILABLE_PROFILES[mainSelection.get()]);
                    } else {
                        mainConfirmed.set(true);
                    }
                }
            });

            MenuHelper.handleControllerInput(this, gamepad2, !subConfirmed.get(), () -> {
                if (gamepad2.dpad_up) {
                    subSelection.set((subSelection.get() - 1 + options.length) % options.length);
                } else if (gamepad2.dpad_down) {
                    subSelection.set((subSelection.get() + 1) % options.length);
                } else if (gamepad2.a) {
                    if (subSelection.get() < Settings.AVAILABLE_PROFILES.length) {
                        subProfile.set(Settings.AVAILABLE_PROFILES[subSelection.get()]);
                    } else {
                        subConfirmed.set(true);
                    }
                }
            });

            // Display selections
            telemetry.addLine("\nSelected Profiles:");
            telemetry.addData("Main Controller", mainProfile.get().name + (mainConfirmed.get() ? " (Confirmed)" : ""));
            telemetry.addData("Sub Controller", subProfile.get().name + (subConfirmed.get() ? " (Confirmed)" : ""));

            // Check for menu completion
            if (mainConfirmed.get() && subConfirmed.get()) {
                menuActive = false;
            }

            telemetry.update();
        }

        // Initialize robot systems
        DynamicInput dynamicInput = new DynamicInput(gamepad1, gamepad2, mainProfile.get(), subProfile.get());
        BaseRobot baseRobot = new BaseRobot(hardwareMap, dynamicInput, this, telemetry);

        // Main loop
        waitForStart();
        while (opModeIsActive()) {
            baseRobot.driveGamepads();
            if (Settings.Deploy.ODOMETRY) {
                baseRobot.odometry.update();
            }
        }
        baseRobot.shutDown();
    }

}
