package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
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

@Autonomous(name = "Meet 3 Auto", group = "Autonomous")
public class MeetThreeAuto extends LinearOpMode {
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

        run(startingPosition);
    }

    public void run(StartingPosition sp) {
        baseRobot.arm.wrist.setPosition(Wrist.Position.VERTICAL);
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
                TrajectoryActionBuilder previousTrajectory = gameLoopSetup(sp, ChamberHeight.LOW);
                while (30 - baseRobot.parentOp.getRuntime() > (Settings.ms_needed_to_park / 1000)) {
                    previousTrajectory = gameLoop(sp, previousTrajectory, ChamberHeight.LOW);
                }
                baseRobot.logger.update("Autonomous phase", "Parking");
                previousTrajectory = gameLoopEnd(sp, previousTrajectory);
                baseRobot.logger.update("Autonomous phase", "Victory is ours");
                break;
        }
    }

    public TrajectoryActionBuilder gameLoopSetup(StartingPosition sp, ChamberHeight chamberHeight) {
        baseRobot.logger.update("Autonomous phase", "Placing initial specimen on chamber");
        TrajectoryActionBuilder placingTrajectory = getPlacingTrajectory(sp, roadRunner.actionBuilder(initialPose));

        Actions.runBlocking(
                new SequentialAction(
                        placingTrajectory.build(),
                        placeChamber()));

        return placingTrajectory;
    }

    public TrajectoryActionBuilder gameLoop(StartingPosition sp, TrajectoryActionBuilder previousTrajectory,
            ChamberHeight chamberHeight) {
        baseRobot.logger.update("Autonomous phase", "Grabbing next specimen");
        previousTrajectory = getNextSpecimen(sp, previousTrajectory);
        baseRobot.logger.update("Autonomous phase", "Placing next specimen");
        previousTrajectory = placeNextSpecimenOnChamber(sp, previousTrajectory, ChamberHeight.HIGH);
        return previousTrajectory;
    }

    public TrajectoryActionBuilder gameLoopEnd(StartingPosition sp, TrajectoryActionBuilder previousPose) {
        TrajectoryActionBuilder parkingTrajectory = getParkingTrajectory(sp, previousPose);

        Actions.runBlocking(
                new SequentialAction(
                        parkingTrajectory.build()));

        return parkingTrajectory;
    }

    /**
     * Enum defining possible chamber heights for scoring
     */
    public enum ChamberHeight {
        /** Lower scoring position */
        LOW,
        /** Upper scoring position */
        HIGH
    }

    public class PlaceChamber implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            baseRobot.arm.wrist.setPosition(Wrist.Position.VERTICAL);
            baseRobot.arm.wrist.setPosition(Wrist.Position.HORIZONTAL);
            baseRobot.arm.intake.outtake();
            pause(200);
            baseRobot.arm.intake.stop();
            return false;
        }
    }

    public Action placeChamber() {
        return new PlaceChamber();
    }

    public class GrabSpecimenFromHumanPlayer implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            baseRobot.arm.wrist.setPosition(Wrist.Position.HORIZONTAL);
            baseRobot.arm.intake.intake();
            baseRobot.arm.wrist.setPosition(Wrist.Position.VERTICAL);
            return false;
        }
    }

    public Action grabSpecimenFromHP() {
        return new GrabSpecimenFromHumanPlayer();
    }

    public TrajectoryActionBuilder placeNextSpecimenOnChamber(StartingPosition sp,
            TrajectoryActionBuilder previousTrajectory, ChamberHeight mode) {
        TrajectoryActionBuilder placingTrajectory = getPlacingTrajectory(sp, previousTrajectory);

        Actions.runBlocking(
                new SequentialAction(
                        placingTrajectory.build(),
                        placeChamber()));

        return placingTrajectory;
    }

    public TrajectoryActionBuilder getNextSpecimen(StartingPosition sp, TrajectoryActionBuilder previousTrajectory) {
        Actions.runBlocking(
                new SequentialAction(
                        getHPTrajectory(sp, previousTrajectory).build(),
                        grabSpecimenFromHP()));
        return getHPTrajectory(sp, previousTrajectory);
    }

    public void immediatelyPlace(StartingPosition sp) {
        TrajectoryActionBuilder placingTrajectory = getPlacingTrajectory(sp, roadRunner.actionBuilder(initialPose));
        TrajectoryActionBuilder parkingTrajectory = getParkingTrajectory(sp, placingTrajectory.endTrajectory().fresh());

        Actions.runBlocking(
                new SequentialAction(
                        placingTrajectory.build(),
                        placeChamber(),
                        parkingTrajectory.build()));
    }

    public void justPark(StartingPosition sp) {
        TrajectoryActionBuilder trajectory;
        switch (sp) {
            case RED_LEFT:
                trajectory = roadRunner.actionBuilder(Settings.Autonomous.FieldPositions.RED_LEFT_INITIAL_POSE)
                        .strafeTo(Settings.Autonomous.FieldPositions.RED_LEFT_JUST_PARK_VEC);
                break;
            case BLUE_LEFT:
                trajectory = roadRunner.actionBuilder(Settings.Autonomous.FieldPositions.BLUE_LEFT_INITIAL_POSE)
                        .strafeTo(Settings.Autonomous.FieldPositions.BLUE_LEFT_JUST_PARK_VEC);
                break;
            case RED_RIGHT:
                trajectory = roadRunner.actionBuilder(Settings.Autonomous.FieldPositions.RED_RIGHT_INITIAL_POSE)
                        .strafeTo(Settings.Autonomous.FieldPositions.RED_RIGHT_JUST_PARK_VEC);
                break;
            case BLUE_RIGHT:
            default:
                trajectory = roadRunner.actionBuilder(Settings.Autonomous.FieldPositions.BLUE_RIGHT_INITIAL_POSE)
                        .strafeTo(Settings.Autonomous.FieldPositions.BLUE_RIGHT_JUST_PARK_VEC);
                break;
        }
        Actions.runBlocking(
                new SequentialAction(
                        trajectory.build()));
    }

    private TrajectoryActionBuilder getParkingTrajectory(MeetThreeAuto.StartingPosition sp,
            TrajectoryActionBuilder previousTrajectory) {
        // Helper method to get parking trajectory based on starting position
        switch (sp) {
            case RED_LEFT:
                return previousTrajectory.strafeTo(Settings.Autonomous.FieldPositions.RED_PARK_MIDDLEMAN)
                        .strafeToLinearHeading(Settings.Autonomous.FieldPositions.RED_LEFT_PARK_POSE.position,
                                Settings.Autonomous.FieldPositions.RED_LEFT_PARK_POSE.heading);
            case RED_RIGHT:
                return previousTrajectory.strafeTo(Settings.Autonomous.FieldPositions.RED_PARK_MIDDLEMAN)
                        .strafeToLinearHeading(Settings.Autonomous.FieldPositions.RED_RIGHT_PARK_POSE.position,
                                Settings.Autonomous.FieldPositions.RED_RIGHT_PARK_POSE.heading);
            case BLUE_LEFT:
                return previousTrajectory.strafeTo(Settings.Autonomous.FieldPositions.BLUE_PARK_MIDDLEMAN)
                        .strafeToLinearHeading(Settings.Autonomous.FieldPositions.BLUE_LEFT_PARK_POSE.position,
                                Settings.Autonomous.FieldPositions.BLUE_LEFT_PARK_POSE.heading);
            case BLUE_RIGHT:
                return previousTrajectory.strafeTo(Settings.Autonomous.FieldPositions.BLUE_PARK_MIDDLEMAN)
                        .strafeToLinearHeading(Settings.Autonomous.FieldPositions.BLUE_RIGHT_PARK_POSE.position,
                                Settings.Autonomous.FieldPositions.BLUE_RIGHT_PARK_POSE.heading);
            default:
                return previousTrajectory;
        }
    }

    private TrajectoryActionBuilder getHPTrajectory(StartingPosition sp, TrajectoryActionBuilder previousTrajectory) {
        // Helper method to get human player trajectory based on starting position
        switch (sp) {
            case RED_LEFT:
            case RED_RIGHT:
                return previousTrajectory
                        .splineTo(Settings.Autonomous.FieldPositions.RED_HP_POSE.position,
                                Settings.Autonomous.FieldPositions.RED_HP_POSE.heading);
            case BLUE_LEFT:
            case BLUE_RIGHT:
                return previousTrajectory
                        .splineTo(Settings.Autonomous.FieldPositions.BLUE_HP_POSE.position,
                                Settings.Autonomous.FieldPositions.BLUE_HP_POSE.heading);
            default:
                return previousTrajectory;
        }
    }

    private TrajectoryActionBuilder getPlacingTrajectory(StartingPosition sp,
            TrajectoryActionBuilder previousTrajectory) {
        // Helper method to get placing trajectory based on starting position
        switch (sp) {
            case RED_LEFT:
                return previousTrajectory
                        .strafeTo(Settings.Autonomous.FieldPositions.RED_LEFT_PLACE_POSE.position);
            case RED_RIGHT:
                return previousTrajectory
                        .strafeTo(Settings.Autonomous.FieldPositions.RED_RIGHT_PLACE_POSE.position);
            case BLUE_LEFT:
                return previousTrajectory
                        .strafeTo(Settings.Autonomous.FieldPositions.BLUE_LEFT_PLACE_POSE.position);
            case BLUE_RIGHT:
                return previousTrajectory
                        .strafeTo(Settings.Autonomous.FieldPositions.BLUE_RIGHT_PLACE_POSE.position);
            default:
                return previousTrajectory;
        }
    }

    // Define an enum for starting positions
    public enum StartingPosition {
        RED_LEFT, RED_RIGHT, BLUE_LEFT, BLUE_RIGHT
    }

    private void pause(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}