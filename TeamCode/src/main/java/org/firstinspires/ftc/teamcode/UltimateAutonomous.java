package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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

@Autonomous(name = "THE ULTIMATE AUTONOMOUS", group = "Autonomous")
public class UltimateAutonomous extends LinearOpMode {
    StartingPosition startingPosition = StartingPosition.RIGHT;

    private static final String[] MENU_OPTIONS = {
            "Right", "Left", "Confirm"
    };

    private BaseRobot baseRobot;

    private MecanumDrive roadRunner;

    private Pose2d initialPose;

    public AdaptiveCalibration adaptiveCalibration;

    public static Settings.Deploy.AutonomousMode autonomousMode;

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
                TrajectoryActionBuilder previousChamberTrajectory = gameLoopSetup(sp, PlacementHeight.CHAMBER_HIGH);
                int phase = 0;
                while (30 - baseRobot.parentOp.getRuntime() > (Settings.ms_needed_to_park / 1000)) {
                    phase++;
                    adaptiveCalibration.calibrateRuntime(new AdaptiveCalibration.RuntimeCalibrationPayload(), roadRunner);
                    if(phase == 1) {
                        previousChamberTrajectory = pushSamples(sp, previousChamberTrajectory);
                    } else {
                        previousChamberTrajectory = placeLoop(sp, previousChamberTrajectory, PlacementHeight.CHAMBER_HIGH);
                    }
                }
                baseRobot.logger.update("Autonomous phase", "Parking");
                gameLoopEnd(sp, previousChamberTrajectory);
                baseRobot.logger.update("Autonomous phase", "Victory is ours");
                break;

            case BASKET:
                TrajectoryActionBuilder previousBasketTrajectory = gameLoopSetup(sp, PlacementHeight.BASKET_HIGH);
                while (30 - baseRobot.parentOp.getRuntime() > (Settings.ms_needed_to_park / 1000)) {
                    adaptiveCalibration.calibrateRuntime(new AdaptiveCalibration.RuntimeCalibrationPayload(), roadRunner);
                    previousBasketTrajectory = placeLoop(sp, previousBasketTrajectory, PlacementHeight.BASKET_HIGH);
                }
                baseRobot.logger.update("Autonomous phase", "Parking");
                gameLoopEnd(sp, previousBasketTrajectory);
                baseRobot.logger.update("Autonomous phase", "Victory is ours");
                break;
        }
    }

    public TrajectoryActionBuilder gameLoopSetup(StartingPosition sp, PlacementHeight chamberHeight) {
        baseRobot.logger.update("Autonomous phase", "Placing initial specimen on chamber");
        TrajectoryActionBuilder placingTrajectory = getPlacingTrajectory(sp, roadRunner.actionBuilder(initialPose));
        TrajectoryActionBuilder unhookTrajectory = getUnhookTrajectory(sp, placingTrajectory);
        TrajectoryActionBuilder sampleTrajectory = pushSamples(sp, placingTrajectory);

        baseRobot.outtake.verticalSlide.setPosition(ViperSlide.VerticalPosition.HIGH_RUNG);

        Actions.runBlocking(
                new SequentialAction(
                        placingTrajectory.build(),
                        hookChamber(),
                        unhookChamber(),
                        sampleTrajectory.build()
                )
        );

        return sampleTrajectory;
    }

    public TrajectoryActionBuilder placeLoop(StartingPosition sp, TrajectoryActionBuilder previousTrajectory,
                                             PlacementHeight placementHeight) {
        baseRobot.telemetry.addData("Autonomous phase", "Grabbing next specimen");
        baseRobot.telemetry.update();
        baseRobot.outtake.linkage.setPosition(Linkage.Position.PLACE_BACKWARD);
        previousTrajectory = getNextSpecimen(sp, previousTrajectory);
        baseRobot.logger.update("Autonomous phase", "Placing next specimen");
        switch (placementHeight) {
            case CHAMBER_LOW:
            case CHAMBER_HIGH:
                previousTrajectory = placeNextSpecimenOnChamber(sp, previousTrajectory, placementHeight);
                break;
            case BASKET_LOW:
            case BASKET_HIGH:
                previousTrajectory = placeNextSampleInBasket(sp, previousTrajectory, placementHeight);
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
            baseRobot.outtake.claw.close();
            baseRobot.outtake.verticalSlide.setPosition(ViperSlide.VerticalPosition.HIGH_RUNG);
            baseRobot.outtake.linkage.setPosition(Linkage.Position.PLACE_FORWARD);
            pause(500);
            baseRobot.outtake.verticalSlide.setPosition(ViperSlide.VerticalPosition.HIGH_RUNG.getValue() + 800);
            pause(500);
            baseRobot.outtake.linkage.setPosition(Linkage.Position.PLACE_BACKWARD);
            baseRobot.outtake.claw.open();
            baseRobot.outtake.verticalSlide.setPosition(ViperSlide.VerticalPosition.TRANSFER);
            return false;
        }
    }

    public Action hookChamber() {
        return new HookChamber();
    }

    public class UnhookChamber implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            pause(300);
            baseRobot.outtake.claw.open();
            pause(300);
            baseRobot.outtake.verticalSlide.setPosition(ViperSlide.VerticalPosition.TRANSFER);
            baseRobot.outtake.linkage.setPosition(Linkage.Position.TRANSFER);
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
            pause(100);
            baseRobot.outtake.linkage.setPosition(Linkage.Position.PLACE_FORWARD);
            baseRobot.outtake.verticalSlide.setPosition(ViperSlide.VerticalPosition.HIGH_RUNG);
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
            TrajectoryActionBuilder previousTrajectory, PlacementHeight placementHeight) {
        TrajectoryActionBuilder placingTrajectory = getPlacingTrajectory_2(sp, previousTrajectory);

        Actions.runBlocking(
                new SequentialAction(
                        placingTrajectory.build(),
                        hookChamber()
                )
        );
        return placingTrajectory;
    }

    public TrajectoryActionBuilder placeNextSampleInBasket(StartingPosition sp,
            TrajectoryActionBuilder previousTrajectory, PlacementHeight placementHeight) {
        TrajectoryActionBuilder basketTrajectory = getBasketTrajectory(sp, previousTrajectory);

        Actions.runBlocking(
                new SequentialAction(
                        basketTrajectory.build(),
                        hookChamber()));

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
        TrajectoryActionBuilder placingTrajectory = getPlacingTrajectory(sp, roadRunner.actionBuilder(initialPose));
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

    private TrajectoryActionBuilder getParkingTrajectory(UltimateAutonomous.StartingPosition sp,
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
                .waitSeconds(1)
                .lineToY(Settings.Autonomous.FieldPositions.HP_POSE.position.y - 12)
                .waitSeconds(0.5);
    }

    private TrajectoryActionBuilder getPlacingTrajectory(StartingPosition sp,
                                                         TrajectoryActionBuilder previousTrajectory) {
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
    private TrajectoryActionBuilder getPlacingTrajectory_2(StartingPosition sp,
                                                         TrajectoryActionBuilder previousTrajectory) {
        // Helper method to get placing trajectory based on starting position
        switch (sp) {
            case LEFT:
                return previousTrajectory.endTrajectory().fresh()
                        .strafeToLinearHeading(Settings.Autonomous.FieldPositions.LEFT_CHAMBER_POSE.position,
                                Settings.Autonomous.FieldPositions.LEFT_CHAMBER_POSE.heading);
            case RIGHT:
                return previousTrajectory.endTrajectory().fresh()
                        .strafeToLinearHeading(Settings.Autonomous.FieldPositions.RIGHT_CHAMBER_POSE_2.position,
                                Settings.Autonomous.FieldPositions.RIGHT_CHAMBER_POSE_2.heading);
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

    private TrajectoryActionBuilder getBasketTrajectory(StartingPosition sp,
            TrajectoryActionBuilder previousTrajectory) {
        return previousTrajectory.endTrajectory().fresh();
//                .strafeToLinearHeading(Settings.Autonomous.FieldPositions.BASKET_POSE.position,
//                        Settings.Autonomous.FieldPositions.BASKET_POSE.heading);
    }

    private TrajectoryActionBuilder pushSamples(StartingPosition sp, TrajectoryActionBuilder previousTrajectory){
        switch (sp) {
            case LEFT:
            case RIGHT:
                return previousTrajectory.endTrajectory().fresh()
                        // gets in front of the first on field sample and pushes it back
                        .setTangent(Math.toRadians(270))
                        .strafeTo(Settings.Autonomous.FieldPositions.RED_RIGHT_SAMPLE_MIDDLEMAN)
                        .splineToSplineHeading(new Pose2d(Settings.Autonomous.FieldPositions.RED_SAMPLE_PUSH_POSE_1.position,
                                Settings.Autonomous.FieldPositions.RED_SAMPLE_PUSH_POSE_1.heading), Math.toRadians(270))
                        .lineToY(Settings.Autonomous.FieldPositions.RED_SAMPLE_PUSH_POSE_1.position.y-37)
                        // this is the same thing for the second sample
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(Settings.Autonomous.FieldPositions.RED_SAMPLE_PUSH_POSE_2.position,
                                Settings.Autonomous.FieldPositions.RED_SAMPLE_PUSH_POSE_2.heading), Math.toRadians(270))
                        .lineToY(Settings.Autonomous.FieldPositions.RED_SAMPLE_PUSH_POSE_2.position.y-37);
            default:
                return previousTrajectory.endTrajectory().fresh();
        }
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