package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.Wrist;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.systems.DynamicInput;
import org.firstinspires.ftc.teamcode.systems.Logger;
import org.firstinspires.ftc.teamcode.utils.MenuHelper;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

@Autonomous(name = "Ben's Secret Sauce", group = "Autonomous")
public class WroteThisOpFromScratchMainlyBecauseTheOthersWereTooCloggedButAlsoDueToTheFactThatIAmJustThatGuy
        extends LinearOpMode {
    StartingPosition startingPosition = StartingPosition.RED_LEFT;

    private static final String[] MENU_OPTIONS = {
            "Red Left", "Red Right", "Blue Left", "Blue Right", "Confirm"
    };

    private BaseRobot baseRobot;

    private MecanumDrive roadRunner;

    private Pose2d initialPose;

    private FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        DynamicInput dynamicInput = new DynamicInput(gamepad1, gamepad2,
                Settings.DEFAULT_PROFILE, Settings.DEFAULT_PROFILE);
        baseRobot = new BaseRobot(hardwareMap, dynamicInput, this, telemetry);
        dashboard = FtcDashboard.getInstance();

        AtomicBoolean menuActive = new AtomicBoolean(true);
        AtomicInteger currentSelection = new AtomicInteger(0);

        while (!isStarted() && !isStopRequested() && menuActive.get()) {
            telemetry.addLine("=== Autonomous Configuration ===");
            telemetry.addLine("\nSelect Starting Position:");

            MenuHelper.displayMenuOptions(telemetry, MENU_OPTIONS, currentSelection.get());

            MenuHelper.handleControllerInput(this, gamepad1, true, () -> {
                if (gamepad1.dpad_up) {
                    currentSelection.set((currentSelection.get() - 1 + MENU_OPTIONS.length) % MENU_OPTIONS.length);
                } else if (gamepad1.dpad_down) {
                    currentSelection.set((currentSelection.get() + 1) % MENU_OPTIONS.length);
                } else if (gamepad1.a) {
                    if (currentSelection.get() < MENU_OPTIONS.length - 1) {
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

            telemetry.addLine("\nSelected Configuration:");
            telemetry.addData("Position",
                    (startingPosition != null) ? startingPosition.name() : "Not selected");
            telemetry.update();
        }

        telemetry.addData("Status", "Ready to start as" + startingPosition.name());
        telemetry.update();

        // Initialize the roadRunner's pose based on the starting position
        switch (startingPosition) {
            case RED_LEFT:
                initialPose = Settings.Autonomous.FieldPositions.RED_LEFT_INITIAL_POSE;
                break;
            case RED_RIGHT:
                initialPose = Settings.Autonomous.FieldPositions.RED_RIGHT_INITIAL_POSE;
                break;
            case BLUE_LEFT:
                initialPose = Settings.Autonomous.FieldPositions.BLUE_LEFT_INITIAL_POSE;
                break;
            case BLUE_RIGHT:
                initialPose = Settings.Autonomous.FieldPositions.BLUE_RIGHT_INITIAL_POSE;
                break;
            default:
                initialPose = new Pose2d(0, 0, 0); // Fallback
                baseRobot.logger.add("Invalid starting position: " + startingPosition, Logger.LogType.PERMANENT);
        }

        roadRunner = new MecanumDrive(hardwareMap, initialPose);
        waitForStart();

        try {
            run(startingPosition);
        } catch (RuntimeException e) {
            // Shutdown handled by ShutdownManager
        }
    }

    public void run(StartingPosition sp) {
        switch (Settings.Deploy.AUTONOMOUS_MODE) {
            case JUST_PARK:
                baseRobot.logger.update("Autonomous phase", "Parking due to deploy flag");
                justPark(sp);
                return;
            case JUST_PLACE:
                baseRobot.logger.update("Autonomous phase", "Placing due to deploy flag");
                immediatelyPlace(sp);
                return;
            case FULL:
                gameLoopSetup(sp, MainAuto.ChamberHeight.LOW);
                while (30 - baseRobot.parentOp.getRuntime() > (Settings.ms_needed_to_park / 1000)) {
                    gameLoop(sp, MainAuto.ChamberHeight.LOW);
                }
                baseRobot.logger.update("Autonomous phase", "Parking");
                gameLoopEnd(sp);
                baseRobot.logger.update("Autonomous phase", "VICTORY!!!");
                if (Settings.Deploy.VICTORY) {
                    victory();
                }
                break;
        }
    }

    private TrajectoryActionBuilder getPlacingTrajectory(StartingPosition sp) {
        // Helper method to get placing trajectory based on starting position
        switch (sp) {
            case RED_LEFT:
                return roadRunner.actionBuilder(initialPose)
                        .splineTo(Settings.Autonomous.FieldPositions.RED_LEFT_PLACE_POSE.position,
                                Settings.Autonomous.FieldPositions.RED_LEFT_PLACE_POSE.heading);
            case RED_RIGHT:
                return roadRunner.actionBuilder(initialPose)
                        .splineTo(Settings.Autonomous.FieldPositions.RED_RIGHT_PLACE_POSE.position,
                                Settings.Autonomous.FieldPositions.RED_RIGHT_PLACE_POSE.heading);
            case BLUE_LEFT:
                return roadRunner.actionBuilder(initialPose)
                        .splineTo(Settings.Autonomous.FieldPositions.BLUE_LEFT_PLACE_POSE.position,
                                Settings.Autonomous.FieldPositions.BLUE_LEFT_PLACE_POSE.heading);
            case BLUE_RIGHT:
                return roadRunner.actionBuilder(initialPose)
                        .splineTo(Settings.Autonomous.FieldPositions.BLUE_RIGHT_PLACE_POSE.position,
                                Settings.Autonomous.FieldPositions.BLUE_RIGHT_PLACE_POSE.heading);
            default:
                return roadRunner.actionBuilder(initialPose);
        }
    }

    public void gameLoopSetup(StartingPosition sp, MainAuto.ChamberHeight chamberHeight) {
        baseRobot.logger.update("Autonomous phase", "Placing initial specimen on chamber");
        TrajectoryActionBuilder placingTrajectory = getPlacingTrajectory(sp);

        Actions.runBlocking(
                new SequentialAction(
                        placingTrajectory.build(),
                        placeChamber()));
    }

    public void gameLoop(StartingPosition sp, MainAuto.ChamberHeight chamberHeight) {
        baseRobot.logger.update("Autonomous phase", "Grabbing next specimen");
        getNextSpecimen(sp);
        baseRobot.logger.update("Autonomous phase", "Placing next specimen");
        placeNextSpecimenOnChamber(sp, MainAuto.ChamberHeight.HIGH);
    }

    public void justPark(StartingPosition sp) {
        TrajectoryActionBuilder parkingTrajectory;

        switch (sp) {
            case RED_LEFT:
                parkingTrajectory = roadRunner.actionBuilder(initialPose)
                        .lineToX(40);
                break;
            case RED_RIGHT:
                parkingTrajectory = roadRunner.actionBuilder(initialPose)
                        .lineToX(60);
                break;
            case BLUE_LEFT:
                parkingTrajectory = roadRunner.actionBuilder(initialPose)
                        .lineToX(-40);
                break;
            case BLUE_RIGHT:
                parkingTrajectory = roadRunner.actionBuilder(initialPose)
                        .lineToX(-60);
                break;
            default:
                parkingTrajectory = roadRunner.actionBuilder(initialPose);

        }

        Actions.runBlocking(
                new SequentialAction(
                        parkingTrajectory.build()));
    }

    public class PlaceChamber implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            baseRobot.arm.wrist.setPosition(Wrist.Position.VERTICAL);
            sleep(500);
            baseRobot.arm.wrist.setPosition(Wrist.Position.HORIZONTAL);
            sleep(200);
            baseRobot.arm.intake.outtake();
            sleep(50);
            return false;
        }
    }

    public Action placeChamber() {
        return new PlaceChamber();
    }

    public class GrabSpecimenFromDaouda implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            baseRobot.arm.wrist.setPosition(Wrist.Position.HORIZONTAL);
            sleep(200);
            baseRobot.arm.intake.intake();
            sleep(50);
            baseRobot.arm.wrist.setPosition(Wrist.Position.VERTICAL);
            return false;
        }
    }

    public Action grabSpecimenFromDaouda() {
        return new GrabSpecimenFromDaouda();
    }

    public void immediatelyPlace(StartingPosition sp) {
        TrajectoryActionBuilder placingTrajectory;
        TrajectoryActionBuilder parkingTrajectory;

        switch (sp) {
            case RED_LEFT:
                placingTrajectory = roadRunner.actionBuilder(initialPose)
                        .splineTo(Settings.Autonomous.FieldPositions.RED_LEFT_PLACE_POSE.position,
                                Settings.Autonomous.FieldPositions.RED_LEFT_PLACE_POSE.heading);
                parkingTrajectory = roadRunner.actionBuilder(initialPose)
                        .splineTo(Settings.Autonomous.FieldPositions.RED_LEFT_PARK_POSE.position,
                                Settings.Autonomous.FieldPositions.RED_LEFT_PARK_POSE.heading);
                break;
            case RED_RIGHT:
                placingTrajectory = roadRunner.actionBuilder(initialPose)
                        .splineTo(Settings.Autonomous.FieldPositions.RED_RIGHT_PLACE_POSE.position,
                                Settings.Autonomous.FieldPositions.RED_RIGHT_PLACE_POSE.heading);
                parkingTrajectory = roadRunner.actionBuilder(initialPose)
                        .splineTo(Settings.Autonomous.FieldPositions.RED_RIGHT_PARK_POSE.position,
                                Settings.Autonomous.FieldPositions.RED_RIGHT_PARK_POSE.heading);
                break;
            case BLUE_LEFT:
                placingTrajectory = roadRunner.actionBuilder(initialPose)
                        .splineTo(Settings.Autonomous.FieldPositions.BLUE_LEFT_PLACE_POSE.position,
                                Settings.Autonomous.FieldPositions.BLUE_LEFT_PLACE_POSE.heading);
                parkingTrajectory = roadRunner.actionBuilder(initialPose)
                        .splineTo(Settings.Autonomous.FieldPositions.BLUE_LEFT_PARK_POSE.position,
                                Settings.Autonomous.FieldPositions.BLUE_LEFT_PARK_POSE.heading);
                break;
            case BLUE_RIGHT:
                placingTrajectory = roadRunner.actionBuilder(initialPose)
                        .splineTo(Settings.Autonomous.FieldPositions.BLUE_RIGHT_PLACE_POSE.position,
                                Settings.Autonomous.FieldPositions.BLUE_RIGHT_PLACE_POSE.heading);
                parkingTrajectory = roadRunner.actionBuilder(initialPose)
                        .splineTo(Settings.Autonomous.FieldPositions.BLUE_RIGHT_PARK_POSE.position,
                                Settings.Autonomous.FieldPositions.BLUE_RIGHT_PARK_POSE.heading);
                break;
            default:
                placingTrajectory = roadRunner.actionBuilder(initialPose);
                parkingTrajectory = roadRunner.actionBuilder(initialPose);
        }

        Actions.runBlocking(
                new SequentialAction(
                        placingTrajectory.build(),
                        placeChamber(),
                        parkingTrajectory.build()));
    }

    public void placeNextSpecimenOnChamber(StartingPosition sp, MainAuto.ChamberHeight mode) {
        TrajectoryActionBuilder placingTrajectory;
        switch (sp) {
            case RED_LEFT:
                placingTrajectory = roadRunner.actionBuilder(initialPose)
                        .splineTo(Settings.Autonomous.FieldPositions.RED_LEFT_PLACE_POSE.position,
                                Settings.Autonomous.FieldPositions.RED_LEFT_PLACE_POSE.heading);
                break;
            case RED_RIGHT:
                placingTrajectory = roadRunner.actionBuilder(initialPose)
                        .splineTo(Settings.Autonomous.FieldPositions.RED_RIGHT_PLACE_POSE.position,
                                Settings.Autonomous.FieldPositions.RED_RIGHT_PLACE_POSE.heading);
                break;
            case BLUE_LEFT:
                placingTrajectory = roadRunner.actionBuilder(initialPose)
                        .splineTo(Settings.Autonomous.FieldPositions.BLUE_LEFT_PLACE_POSE.position,
                                Settings.Autonomous.FieldPositions.BLUE_LEFT_PLACE_POSE.heading);
                break;
            case BLUE_RIGHT:
                placingTrajectory = roadRunner.actionBuilder(initialPose)
                        .splineTo(Settings.Autonomous.FieldPositions.BLUE_RIGHT_PLACE_POSE.position,
                                Settings.Autonomous.FieldPositions.BLUE_RIGHT_PLACE_POSE.heading);
                break;
            default:
                placingTrajectory = roadRunner.actionBuilder(initialPose);
        }

        Actions.runBlocking(
                new SequentialAction(
                        placingTrajectory.build(),
                        placeChamber()));
    }

    public void gameLoopEnd(StartingPosition sp) {
        TrajectoryActionBuilder parkingTrajectory;

        switch (sp) {
            case RED_LEFT:
                parkingTrajectory = roadRunner.actionBuilder(initialPose)
                        .splineTo(Settings.Autonomous.FieldPositions.RED_LEFT_PARK_POSE.position,
                                Settings.Autonomous.FieldPositions.RED_LEFT_PARK_POSE.heading);
                break;
            case RED_RIGHT:
                parkingTrajectory = roadRunner.actionBuilder(initialPose)
                        .splineTo(Settings.Autonomous.FieldPositions.RED_RIGHT_PARK_POSE.position,
                                Settings.Autonomous.FieldPositions.RED_RIGHT_PARK_POSE.heading);
                break;
            case BLUE_LEFT:
                parkingTrajectory = roadRunner.actionBuilder(initialPose)
                        .splineTo(Settings.Autonomous.FieldPositions.BLUE_LEFT_PARK_POSE.position,
                                Settings.Autonomous.FieldPositions.BLUE_LEFT_PARK_POSE.heading);
                break;
            case BLUE_RIGHT:
                parkingTrajectory = roadRunner.actionBuilder(initialPose)
                        .splineTo(Settings.Autonomous.FieldPositions.BLUE_RIGHT_PARK_POSE.position,
                                Settings.Autonomous.FieldPositions.BLUE_RIGHT_PARK_POSE.heading);
                break;
            default:
                parkingTrajectory = roadRunner.actionBuilder(initialPose);
        }

        Actions.runBlocking(
                new SequentialAction(
                        parkingTrajectory.build()));
    }

    public void victory() {
        return; // ! TODO
    }

    public void getNextSpecimen(StartingPosition sp) {
        TrajectoryActionBuilder grabTrajectory;
        switch (sp) {
            case RED_LEFT:
            case RED_RIGHT:
                grabTrajectory = roadRunner.actionBuilder(initialPose)
                        .splineTo(Settings.Autonomous.FieldPositions.RED_HP_POSE.position,
                                Settings.Autonomous.FieldPositions.RED_HP_POSE.heading);
                break;
            case BLUE_RIGHT:
            case BLUE_LEFT:
                grabTrajectory = roadRunner.actionBuilder(initialPose)
                        .splineTo(Settings.Autonomous.FieldPositions.BLUE_HP_POSE.position,
                                Settings.Autonomous.FieldPositions.BLUE_HP_POSE.heading);
                break;
            default:
                grabTrajectory = roadRunner.actionBuilder(initialPose);

        }

        Actions.runBlocking(
                new SequentialAction(
                        grabTrajectory.build(),
                        grabSpecimenFromDaouda()));
    }

    // Define an enum for starting positions
    public enum StartingPosition {
        RED_LEFT, RED_RIGHT, BLUE_LEFT, BLUE_RIGHT
    }

}