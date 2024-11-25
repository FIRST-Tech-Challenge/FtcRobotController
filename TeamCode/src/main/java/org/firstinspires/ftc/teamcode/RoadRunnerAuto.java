package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.systems.DynamicInput;
import org.firstinspires.ftc.teamcode.systems.Logger;
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
@Autonomous(name = "RoadRunner Autonomous", group = "Autonomous")
public class RoadRunnerAuto extends LinearOpMode {
    /** Selected alliance color (Red/Blue) */
    String color = "Red";

    /** Selected starting position (Left/Right) */
    StartingPosition startingPosition = StartingPosition.RED_LEFT;

    /** Menu options for autonomous configuration */
    private static final String[] MENU_OPTIONS = {
            "Red Left", "Red Right", "Blue Left", "Blue Right", "Confirm"
    };

    /** Base robot instance */
    private BaseRobot baseRobot;

    private MecanumDrive roadRunner; // Assuming you have a RoadRunnerDrive class

    private Pose2d initialPose;

    // Add this line to declare the dashboard variable
    private FtcDashboard dashboard; // Assuming you have a Dashboard class

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
        baseRobot = new BaseRobot(hardwareMap, dynamicInput, this, telemetry);

        // Add initialization status
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Give hardware time to initialize
        sleep(500); // Add a small delay for hardware initialization

        AtomicBoolean menuActive = new AtomicBoolean(true);
        AtomicInteger currentSelection = new AtomicInteger(0);

        // Initialize the dashboard
        dashboard = FtcDashboard.getInstance(); // Initialize the dashboard object

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
                        // Set the starting position based on the selection
                        switch (MENU_OPTIONS[currentSelection.get()]) {
                            case "Red Left":
                                startingPosition = StartingPosition.RED_LEFT;
                                break;
                            case "Red Right":
                                startingPosition = StartingPosition.RED_RIGHT;
                                break;
                            case "Blue Left":
                                startingPosition = StartingPosition.BLUE_LEFT;
                                break;
                            case "Blue Right":
                                startingPosition = StartingPosition.BLUE_RIGHT;
                                break;
                        }
                    } else {
                        menuActive.set(false);
                    }
                }
            });

            // Display current selection
            telemetry.addLine("\nSelected Configuration:");
            telemetry.addData("Position",
                    (color != null && startingPosition != null) ? startingPosition.name() : "Not selected");
            telemetry.update();
        }

        // Add ready status
        telemetry.addData("Status", "Ready to start!");
        telemetry.addData("Configuration", startingPosition.name());
        telemetry.update();

        // Initialize the roadRunner's pose based on the starting position
        switch (startingPosition) {
            case RED_LEFT:
                initialPose = new Pose2d(12, 36, 0); // Example coordinates for Red Left
                break;
            case RED_RIGHT:
                initialPose = new Pose2d(12, 12, Math.PI); // Example coordinates for Red Right
                break;
            case BLUE_LEFT:
                initialPose = new Pose2d(0, 36, Math.PI / 2); // Example coordinates for Blue Left
                break;
            case BLUE_RIGHT:
                initialPose = new Pose2d(-36, 60, Math.toRadians(270));
                break;
            default:
                initialPose = new Pose2d(0, 0, 0); // Fallback
        }

        // Initialize the roadRunner with the determined pose
        roadRunner = new MecanumDrive(hardwareMap, initialPose);

        // Now wait for start
        waitForStart();

        try {
            run(startingPosition);
        } catch (RuntimeException e) {
            // Shutdown handled by ShutdownManager
        }

        // Update the field overlay after starting
        updateFieldOverlay();
    }

    // New method to update the field overlay
    private void updateFieldOverlay() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay()
                .setFill("blue") // Set the color for the robot
                .setStroke("black") // Set the outline color
                .fillCircle(initialPose.position.x, initialPose.position.y, 5); // Draw the robot's position
        // Add more details as needed, like orientation
        packet.put("Robot Orientation", initialPose.heading);
        // Send the packet to the dashboard
        dashboard.sendTelemetryPacket(packet);
    }

    public void run(StartingPosition sp) {
        baseRobot.logger.add("yea man we out here", Logger.LogType.PERMANENT);
        if (Settings.Deploy.AUTONOMOUS_MODE == Settings.Deploy.AutonomousMode.JUST_PARK) {
            baseRobot.logger.update("Autonomous phase", "Parking due to deploy flag");
            immediatelyPark(sp);
            return;
        }

        if (Settings.Deploy.AUTONOMOUS_MODE == Settings.Deploy.AutonomousMode.JUST_PLACE) {
            baseRobot.logger.update("Autonomous phase", "Placing due to deploy flag");
            immediatelyPlace(sp);
            return;
        }
        baseRobot.logger.update("Autonomous phase", "Placing initial specimen on chamber");
        placeOnChamber(sp, MainAuto.ChamberHeight.HIGH);
        while (30 - baseRobot.parentOp.getRuntime() > (Settings.ms_needed_to_park / 1000)) {
            baseRobot.logger.update("Autonomous phase", "Grabbing next specimen");
            getNextSpecimen();
            baseRobot.logger.update("Autonomous phase", "Placing next specimen");
            placeNextSpecimenOnChamber(MainAuto.ChamberHeight.HIGH);
        }
        baseRobot.logger.update("Autonomous phase", "Parking");
        park(sp);
        baseRobot.logger.update("Autonomous phase", "VICTORY!!!");
        if (Settings.Deploy.VICTORY) {
            victory();
        }
    }

    public void immediatelyPark(StartingPosition mode) {
        baseRobot.logger.add("yea bro its mode " + mode.name(), Logger.LogType.PERMANENT);
        sleep(1000);

        switch (mode) {
            case RED_RIGHT:
                roadRunner.actionBuilder(initialPose).splineTo(new Vector2d(60, -60), Math.toRadians(90)).build()
                        .run(new TelemetryPacket());
                break;
            case RED_LEFT:
                baseRobot.logger.update("we are running", "red left");
                roadRunner.actionBuilder(initialPose).splineTo(new Vector2d(200, 36), Math.toRadians(90)).build()
                        .run(new TelemetryPacket());
                sleep(1000);
                break;
            case BLUE_RIGHT:
                roadRunner.actionBuilder(initialPose).splineTo(new Vector2d(60, 60), Math.toRadians(90)).build()
                        .run(new TelemetryPacket());
                break;
            case BLUE_LEFT:
                roadRunner.actionBuilder(initialPose).splineTo(new Vector2d(-60, 60), Math.toRadians(90)).build()
                        .run(new TelemetryPacket());
                break;
        }
    }

    public void immediatelyPlace(StartingPosition mode) {
        return; // ! TODO
    }

    public void placeNextSpecimenOnChamber(MainAuto.ChamberHeight mode) {
        return; // ! TODO
    }

    public void placeOnChamber(StartingPosition mode, MainAuto.ChamberHeight high) {
        return; // ! TODO
    }

    public void park(StartingPosition mode) {
        return; // ! TODO
    }

    public void victory() {
        return; // ! TODO
    }

    public void getNextSpecimen() {
        return; // ! TODO
    }

    // Define an enum for starting positions
    public enum StartingPosition {
        RED_LEFT, RED_RIGHT, BLUE_LEFT, BLUE_RIGHT
    }

}