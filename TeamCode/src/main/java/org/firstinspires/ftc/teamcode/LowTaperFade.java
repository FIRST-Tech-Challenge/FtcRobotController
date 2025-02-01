package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Settings.Autonomous.FieldPositions.BASKET_POSE;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.Linkage;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.ViperSlide;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.Wrist;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.systems.DynamicInput;
import org.firstinspires.ftc.teamcode.systems.Logger;
import org.firstinspires.ftc.teamcode.utils.MenuHelper;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

@Autonomous(name = "What if Conner got a lowwwwtaperfade", group = "Autonomous")
public class LowTaperFade extends LinearOpMode {
    StartingPosition startingPosition = StartingPosition.RIGHT;

    private static final String[] MENU_OPTIONS = {
            "Right", "Left", "Confirm"
    };

    private BaseRobot baseRobot;

    private MecanumDrive roadRunner;

    private Pose2d initialPose;

    public AdaptiveCalibration adaptiveCalibration;

    public static Settings.Deploy.AutonomousMode autonomousMode = Settings.Deploy.AUTONOMOUS_MODE_RIGHT;

    @Override
    public void runOpMode() {
        adaptiveCalibration = AdaptiveCalibration.getInstance();
        DynamicInput dynamicInput = new DynamicInput(gamepad1, gamepad2,
                Settings.DEFAULT_PROFILE, Settings.DEFAULT_PROFILE);
        baseRobot = new BaseRobot(hardwareMap, dynamicInput, this, telemetry);

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
                            case "Left":
                                startingPosition = StartingPosition.LEFT;
                                autonomousMode = Settings.Deploy.AUTONOMOUS_MODE_LEFT;
                                break;
                            case "Right":
                                startingPosition = StartingPosition.RIGHT;
                                autonomousMode = Settings.Deploy.AUTONOMOUS_MODE_RIGHT;
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
            case LEFT:
                initialPose = Settings.Autonomous.FieldPositions.LEFT_INITIAL_POSE;
                break;
            case RIGHT:
                initialPose = Settings.Autonomous.FieldPositions.RIGHT_INITIAL_POSE;
                break;
            default:
                initialPose = new Pose2d(0, 0, 0); // Fallback
                baseRobot.logger.add("Invalid starting position: " + startingPosition, Logger.LogType.PERMANENT);
        }

        roadRunner = new MecanumDrive(hardwareMap, initialPose);
        adaptiveCalibration.initialize(roadRunner);
        waitForStart();

        run(startingPosition);
    }

    public void run(StartingPosition sp) {
        baseRobot.intake.wrist.setPosition(Wrist.Position.VERTICAL);
        switch (autonomousMode) {
            case JUST_PARK:
                baseRobot.logger.update("Autonomous phase", "Parking due to deploy flag");
                justPark(sp);
                return;

            case JUST_PLACE:
                baseRobot.logger.update("Autonomous phase", "Placing due to deploy flag");
                immediatelyPlace(sp);
                return;

            case CHAMBER:
            case BASKET:
            default:
                switch (startingPosition) {
                    case RIGHT:
                        TrajectoryActionBuilder previousChamberTrajectory = gameLoopSetup(sp, PlacementHeight.CHAMBER_HIGH);
                        int phase = 0;
                        while (phase < 2) {
                            phase++;
                            adaptiveCalibration.calibrateRuntime(new AdaptiveCalibration.RuntimeCalibrationPayload(),
                                    roadRunner);
                            previousChamberTrajectory = placeLoop(sp, previousChamberTrajectory, PlacementHeight.CHAMBER_HIGH,
                                    phase);
                        }
                        baseRobot.logger.update("Autonomous phase", "Parking");
                        gameLoopEnd(sp, previousChamberTrajectory);
                        baseRobot.logger.update("Autonomous phase", "Victory is ours");
                        break;
                    case LEFT:
                        TrajectoryActionBuilder previousBasketTrajectory = basketLoopSetup(sp);
                        int basketPhase = 1;
                        while (basketPhase < 4) {
                            basketPhase++;
                            adaptiveCalibration.calibrateRuntime(new AdaptiveCalibration.RuntimeCalibrationPayload(),
                                    roadRunner);
                            previousBasketTrajectory = placeLoop(sp, previousBasketTrajectory, PlacementHeight.CHAMBER_HIGH,
                                    basketPhase);
                        }
                        gameLoopEnd(sp, previousBasketTrajectory);
                        break;

                }
        }
    }

    public TrajectoryActionBuilder gameLoopSetup(StartingPosition sp, PlacementHeight chamberHeight) {
        baseRobot.logger.update("Autonomous phase", "Placing initial specimen on chamber");
        TrajectoryActionBuilder placingTrajectory = getPlacingTrajectory(sp, roadRunner.actionBuilder(initialPose), 0);
        TrajectoryActionBuilder sampleTrajectory = pushSamples(sp, placingTrajectory);
        TrajectoryActionBuilder placingTrajectory2 = getPlacingTrajectory(sp, sampleTrajectory, 0);
        baseRobot.outtake.claw.close();
        baseRobot.outtake.verticalSlide.setPosition(Settings.Hardware.VerticalSlide.HIGH_RUNG_PREP_AUTO);
        baseRobot.outtake.linkage.setPosition(Linkage.Position.PLACE_FORWARD);

        Actions.runBlocking(
                new SequentialAction(
                        placingTrajectory.build(),
                        hookChamber(),
                        unhookChamber(),
                        sampleTrajectory.build(),
                        grabSpecimenFromHP(),
                        placingTrajectory2.build(),
                        hookChamber()));

        return sampleTrajectory;
    }

    public TrajectoryActionBuilder basketLoopSetup(StartingPosition sp) {
        baseRobot.logger.update("Autonomous phase", "Placing initial specimen on chamber");
        TrajectoryActionBuilder basketTrajectory = getBasketTrajectory(sp, roadRunner.actionBuilder(initialPose), 0);
        baseRobot.outtake.claw.close();
        baseRobot.outtake.verticalSlide.setPosition(Settings.Hardware.VerticalSlide.TRANSFER);
        baseRobot.outtake.linkage.setPosition(Linkage.Position.PLACE_FORWARD);
        baseRobot.intake.horizontalSlide.setPosition(ViperSlide.HorizontalPosition.COLLAPSED);

        Actions.runBlocking(
                new SequentialAction(
                        basketTrajectory.build()
                )
        );

        return basketTrajectory;
    }

    public TrajectoryActionBuilder placeLoop(StartingPosition sp, TrajectoryActionBuilder previousTrajectory,
                                             PlacementHeight placementHeight, int blocksScored) {
        baseRobot.telemetry.addData("Autonomous phase", "Grabbing next specimen");
        baseRobot.telemetry.update();
        baseRobot.outtake.verticalSlide.setPosition(ViperSlide.VerticalPosition.TRANSFER);
        baseRobot.outtake.linkage.setPosition(Linkage.Position.PLACE_BACKWARD);
        baseRobot.logger.update("Autonomous phase", "Placing next specimen");
        switch (startingPosition) {
            case RIGHT:
                previousTrajectory = getNextSpecimen(sp, previousTrajectory);
                previousTrajectory = placeNextSpecimenOnChamber(sp, previousTrajectory, placementHeight, blocksScored);
                break;
            case LEFT:
                previousTrajectory = runLeftSpecimenTrajectory(sp, previousTrajectory, blocksScored);
                previousTrajectory = placeNextSampleInBasket(sp, previousTrajectory, blocksScored);
                break;
        }
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
    public enum PlacementHeight {
        /** Lower scoring position */
        CHAMBER_LOW,
        /** Upper scoring position */
        CHAMBER_HIGH,
        BASKET_LOW,
        /** Upper scoring position */
        BASKET_HIGH,

    }

    public class HookChamber implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            baseRobot.outtake.linkage.setPosition(Linkage.Position.PLACE_BACKWARD);
            baseRobot.outtake.verticalSlide.setPosition(ViperSlide.VerticalPosition.HIGH_RUNG.getValue() + 500);
            pause(1000);
            baseRobot.outtake.claw.open();
            return false;
        }
    }

    public Action hookChamber() {
        return new HookChamber();
    }

    public class PlaceBasket implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            baseRobot.outtake.verticalSlide.setPosition(ViperSlide.VerticalPosition.HIGH_BASKET);
            pause(1000);
            baseRobot.outtake.linkage.setPosition(Linkage.Position.PLACE_BACKWARD);
            pause(400);
            baseRobot.outtake.claw.open();
            pause(100);
            baseRobot.outtake.linkage.setPosition(Linkage.Position.PLACE_BACKWARD);
            pause(400);
            baseRobot.outtake.verticalSlide.setPosition(ViperSlide.VerticalPosition.TRANSFER);
            pause(100);

            return false;
        }
    }

    public Action placeBasket() {
        return new PlaceBasket();
    }

    public class UnhookChamber implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            baseRobot.outtake.verticalSlide.setPosition(ViperSlide.VerticalPosition.TRANSFER);
            baseRobot.outtake.linkage.setPosition(Linkage.Position.PLACE_BACKWARD);
            return false;
        }
    }

    public Action unhookChamber() {
        return new UnhookChamber();
    }

    public class GrabSpecimenFromHumanPlayer implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            baseRobot.outtake.claw.close();
            pause(500);
            baseRobot.outtake.linkage.setPosition(Linkage.Position.PLACE_FORWARD);
            baseRobot.outtake.verticalSlide.setPosition(Settings.Hardware.VerticalSlide.HIGH_RUNG_PREP_AUTO);
            return false;
        }
    }

    public Action grabSpecimenFromHP() {
        return new GrabSpecimenFromHumanPlayer();
    }

    public class HangAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            baseRobot.outtake.verticalSlide.setPosition(ViperSlide.VerticalPosition.TRANSFER);
            baseRobot.outtake.linkage.setPosition(Linkage.Position.PLACE_BACKWARD);
            pause(2000);
            return false;
        }
    }

    public Action hang() {
        return new HangAction();
    }

    public TrajectoryActionBuilder placeNextSpecimenOnChamber(StartingPosition sp,
            TrajectoryActionBuilder previousTrajectory, PlacementHeight placementHeight, int specimensScored) {
        TrajectoryActionBuilder placingTrajectory = getPlacingTrajectory(sp, previousTrajectory, specimensScored);
        Actions.runBlocking(
                new SequentialAction(
                        placingTrajectory.build(),
                        hookChamber()
//                        ,unhookChamber()
                ));
        return placingTrajectory;
    }

    public TrajectoryActionBuilder placeNextSampleInBasket(StartingPosition sp,
                                                           TrajectoryActionBuilder previousTrajectory, int basketPhase) {
        TrajectoryActionBuilder basketTrajectory = getBasketTrajectory(sp, previousTrajectory, basketPhase);

        Actions.runBlocking(
                new SequentialAction(
                        basketTrajectory.build(),
                        placeBasket()));

        return basketTrajectory;
    }

    public TrajectoryActionBuilder getNextSpecimen(StartingPosition sp, TrajectoryActionBuilder previousTrajectory) {
        TrajectoryActionBuilder hpTrajectory = getHPTrajectory(sp, previousTrajectory);
        Actions.runBlocking(
                new SequentialAction(
                        hpTrajectory.build(),
                        grabSpecimenFromHP()));
        return hpTrajectory;
    }

    public void immediatelyPlace(StartingPosition sp) {
        TrajectoryActionBuilder placingTrajectory = getPlacingTrajectory(sp, roadRunner.actionBuilder(initialPose), 0);
        TrajectoryActionBuilder unhookTrajectory = getUnhookTrajectory(sp, placingTrajectory);
        TrajectoryActionBuilder parkingTrajectory = getParkingTrajectory(sp, unhookTrajectory);

        baseRobot.outtake.verticalSlide.setPosition(ViperSlide.VerticalPosition.HIGH_RUNG);

        Actions.runBlocking(
                new SequentialAction(
                        placingTrajectory.build(),
                        hookChamber(),
                        unhookTrajectory.build(),
                        unhookChamber(),
                        parkingTrajectory.build(),
                        hang()));
    }

    public void justPark(StartingPosition sp) {
        TrajectoryActionBuilder trajectory;
        switch (sp) {
            case LEFT:
                trajectory = roadRunner.actionBuilder(Settings.Autonomous.FieldPositions.LEFT_INITIAL_POSE)
                        .strafeTo(Settings.Autonomous.FieldPositions.LEFT_JUST_PARK_VEC);
                break;
            case RIGHT:
            default:
                trajectory = roadRunner.actionBuilder(Settings.Autonomous.FieldPositions.RIGHT_INITIAL_POSE)
                        .strafeTo(Settings.Autonomous.FieldPositions.RIGHT_JUST_PARK_VEC);
                break;
        }
        Actions.runBlocking(
                new SequentialAction(
                        trajectory.build()));
    }

    private TrajectoryActionBuilder getParkingTrajectory(LowTaperFade.StartingPosition sp,
                                                         TrajectoryActionBuilder previousTrajectory) {
        // Helper method to get parking trajectory based on starting position
        switch (sp) {
            case LEFT:
                return previousTrajectory.endTrajectory().fresh()
                        .strafeToLinearHeading(Settings.Autonomous.FieldPositions.PARK_MIDDLEMAN,
                                Math.toRadians(90))
                        .strafeTo(Settings.Autonomous.FieldPositions.LEFT_BEFORE_PARK_POSE.position)
                        .turn(Math.toRadians(90))
                        .strafeTo(Settings.Autonomous.FieldPositions.LEFT_PARK_POSE.position);
            case RIGHT:
                return previousTrajectory.endTrajectory().fresh()
                        .strafeTo(Settings.Autonomous.FieldPositions.RIGHT_PARK_POSE.position);
            default:
                return previousTrajectory;
        }
    }

    private TrajectoryActionBuilder getHPTrajectory(StartingPosition sp, TrajectoryActionBuilder previousTrajectory) {
        return previousTrajectory.endTrajectory().fresh()
                .strafeToLinearHeading(Settings.Autonomous.FieldPositions.HP_POSE.position,
                        Settings.Autonomous.FieldPositions.HP_POSE.heading)
                .waitSeconds(.1)
                .lineToY(Settings.Autonomous.FieldPositions.HP_POSE.position.y - 9)
                .waitSeconds(0.5);
    }

    private TrajectoryActionBuilder getPlacingTrajectory(StartingPosition sp,
                                                         TrajectoryActionBuilder previousTrajectory, int specimensScored) {
        // Helper method to get placing trajectory based on starting position
        switch (sp) {
            case LEFT:
                return previousTrajectory.endTrajectory().fresh()
                        .strafeToLinearHeading(Settings.Autonomous.FieldPositions.LEFT_CHAMBER_POSE.position,
                                Settings.Autonomous.FieldPositions.LEFT_CHAMBER_POSE.heading);
            case RIGHT:
                return previousTrajectory.endTrajectory().fresh()

                        .strafeToLinearHeading(Settings.Autonomous.FieldPositions.RIGHT_CHAMBER_POSE.position,
                                Settings.Autonomous.FieldPositions.RIGHT_CHAMBER_POSE.heading);
            default:
                return previousTrajectory.endTrajectory().fresh();
        }
    }

    private TrajectoryActionBuilder getUnhookTrajectory(StartingPosition sp,
                                                        TrajectoryActionBuilder previousTrajectory) {
        // Helper method to get placing trajectory based on starting position
        switch (sp) {
            case LEFT:
                return previousTrajectory.endTrajectory().fresh()
                        .lineToY(Settings.Autonomous.FieldPositions.LEFT_CHAMBER_POSE.position.y - 5);
            case RIGHT:
                return previousTrajectory.endTrajectory().fresh()
                        .lineToY(Settings.Autonomous.FieldPositions.RIGHT_CHAMBER_POSE.position.y - 5);
            default:
                return previousTrajectory.endTrajectory().fresh();
        }
    }

    private TrajectoryActionBuilder runLeftSpecimenTrajectory(StartingPosition sp,
                                                              TrajectoryActionBuilder previousTrajectory, int basketPhase) {
        TrajectoryActionBuilder speciTraj;
        switch (basketPhase) {
            case 1:
                speciTraj = previousTrajectory.endTrajectory().fresh()
                        // gets in front of the first on field sample and pushes it back
                        .strafeToConstantHeading(new Vector2d(Settings.Autonomous.FieldPositions.BASKET_MIDDLEMAN.x, Settings.Autonomous.FieldPositions.BASKET_MIDDLEMAN.y))
                        .lineToY(Settings.Autonomous.FieldPositions.BASKET_MIDDLEMAN.y + 50)
                        .strafeToConstantHeading(new Vector2d(Settings.Autonomous.FieldPositions.LEFT_SAMPLE_1_VEC.x, Settings.Autonomous.FieldPositions.LEFT_SAMPLE_1_VEC.y))
                        .lineToY(Settings.Autonomous.FieldPositions.LEFT_SAMPLE_1_VEC.y - 50)
                        .strafeToConstantHeading(Settings.Autonomous.FieldPositions.LEFT_SAMPLE_1_VEC);
                break;
            case 2:
                speciTraj = previousTrajectory.endTrajectory().fresh()
                        // gets in front of the second on field sample and pushes it back
                        .strafeToConstantHeading(new Vector2d(Settings.Autonomous.FieldPositions.BASKET_MIDDLEMAN.x, Settings.Autonomous.FieldPositions.BASKET_MIDDLEMAN.y))
                        .lineToY(Settings.Autonomous.FieldPositions.BASKET_MIDDLEMAN.y + 50)
                        .strafeToConstantHeading(new Vector2d(Settings.Autonomous.FieldPositions.LEFT_SAMPLE_2_VEC.x, Settings.Autonomous.FieldPositions.LEFT_SAMPLE_2_VEC.y))
                        .lineToY(Settings.Autonomous.FieldPositions.LEFT_SAMPLE_2_VEC.y - 50)
                        .strafeToConstantHeading(Settings.Autonomous.FieldPositions.LEFT_SAMPLE_2_VEC);
                break;
            case 3:
            default:
                speciTraj = previousTrajectory.endTrajectory().fresh()
                        // gets in front of the third on field sample and pushes it back
                        .strafeTo(new Vector2d(Settings.Autonomous.FieldPositions.BASKET_MIDDLEMAN.x, Settings.Autonomous.FieldPositions.BASKET_MIDDLEMAN.y))
                        .lineToY(Settings.Autonomous.FieldPositions.BASKET_MIDDLEMAN.y + 50)
                        .strafeTo(new Vector2d(Settings.Autonomous.FieldPositions.LEFT_SAMPLE_3_VEC.x, Settings.Autonomous.FieldPositions.LEFT_SAMPLE_3_VEC.y))
                        .lineToY(Settings.Autonomous.FieldPositions.LEFT_SAMPLE_3_VEC.y - 50)
                        .strafeTo(Settings.Autonomous.FieldPositions.LEFT_SAMPLE_3_VEC);
                break;
        }

        Actions.runBlocking(
                new SequentialAction(
                        speciTraj.build()));

        return speciTraj;
    }

    private TrajectoryActionBuilder getBasketTrajectory(StartingPosition sp,
                                                        TrajectoryActionBuilder previousTrajectory, int basketPhase) {
        return previousTrajectory.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(BASKET_POSE.position.x, BASKET_POSE.position.y), BASKET_POSE.heading);
    }

    private TrajectoryActionBuilder pushSamples(StartingPosition sp, TrajectoryActionBuilder previousTrajectory) {
        return previousTrajectory.endTrajectory().fresh()
                // gets in front of the first on field sample and pushes it back
                .setTangent(Math.toRadians(90))
                .strafeTo(Settings.Autonomous.FieldPositions.SAMPLE_MIDDLEMAN)
                .splineToLinearHeading(new Pose2d(Settings.Autonomous.FieldPositions.FIRST_PRESET_SAMPLE_POSE.position,
                        Settings.Autonomous.FieldPositions.FIRST_PRESET_SAMPLE_POSE.heading), Math.toRadians(270))
                .lineToY(Settings.Autonomous.FieldPositions.FIRST_PRESET_SAMPLE_POSE.position.y - 50)
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(Settings.Autonomous.FieldPositions.SECOND_PRESET_SAMPLE_POSE.position,
                        Settings.Autonomous.FieldPositions.SECOND_PRESET_SAMPLE_POSE.heading), Math.toRadians(270))
                .lineToY(Settings.Autonomous.FieldPositions.SECOND_PRESET_SAMPLE_POSE.position.y - 60);
    }

    // Define an enum for starting positions
    public enum StartingPosition {
        RIGHT, LEFT
    }

    private void pause(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}