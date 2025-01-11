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

import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.Linkage;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.ViperSlide;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.Wrist;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.systems.DynamicInput;
import org.firstinspires.ftc.teamcode.systems.Logger;
import org.firstinspires.ftc.teamcode.utils.MenuHelper;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

@Autonomous(name = "#1 Deepak Fan", group = "Autonomous")
public class MeetFourAuto extends LinearOpMode {
    StartingPosition startingPosition = StartingPosition.RED_LEFT;

    private static final String[] MENU_OPTIONS = {
            "Red Left", "Red Right", "Blue Left", "Blue Right", "Confirm"
    };

    private BaseRobot baseRobot;

    private MecanumDrive roadRunner;

    private Pose2d initialPose;

    private FtcDashboard dashboard;

    public AdaptiveCalibration adaptiveCalibration;

    @Override
    public void runOpMode() {
        adaptiveCalibration = AdaptiveCalibration.getInstance();
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
        adaptiveCalibration.initialize(roadRunner);
        waitForStart();

        run(startingPosition);
    }

    public void run(StartingPosition sp) {
        baseRobot.intake.wrist.setPosition(Wrist.Position.VERTICAL);
        switch (Settings.Deploy.AUTONOMOUS_MODE) {
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
                while (30 - baseRobot.parentOp.getRuntime() > (Settings.ms_needed_to_park / 1000)) {
                    adaptiveCalibration.calibrateRuntime(new AdaptiveCalibration.RuntimeCalibrationPayload(), roadRunner);
                    previousChamberTrajectory = gameLoop(sp, previousChamberTrajectory, PlacementHeight.CHAMBER_HIGH);
                }
                baseRobot.logger.update("Autonomous phase", "Parking");
                gameLoopEnd(sp, previousChamberTrajectory);
                baseRobot.logger.update("Autonomous phase", "Victory is ours");
                break;

            case BASKET:
                TrajectoryActionBuilder previousBasketTrajectory = gameLoopSetup(sp, PlacementHeight.BASKET_HIGH);
                while (30 - baseRobot.parentOp.getRuntime() > (Settings.ms_needed_to_park / 1000)) {
                    adaptiveCalibration.calibrateRuntime(new AdaptiveCalibration.RuntimeCalibrationPayload(), roadRunner);
                    previousBasketTrajectory = gameLoop(sp, previousBasketTrajectory, PlacementHeight.BASKET_HIGH);
                }
                baseRobot.logger.update("Autonomous phase", "Parking");
                gameLoopEnd(sp, previousBasketTrajectory);
                baseRobot.logger.update("Autonomous phase", "Victory is ours");
                break;
        }
    }

    public void goober() {
        Pose2d initialPose = new Pose2d(11.5, -60, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // vision here that outputs position
        int visionOutputPosition = 1;


        TrajectoryActionBuilder MoveSampleToHumanPlayerZone = drive.actionBuilder(new Pose2d(11.5, -60, Math.toRadians(270))).endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(36, -35), Math.toRadians(90))
                .strafeTo(new Vector2d(38,-13))
                .strafeTo(new Vector2d(50,-13))
                .strafeTo(new Vector2d(50,-50))
                .strafeTo(new Vector2d(50,-13))
                .strafeTo(new Vector2d(60,-13))
                .strafeTo(new Vector2d(60,-50));
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3);
        Action PlaceSample = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(5, -28), Math.toRadians(270))
                .build();
        Action trajectoryActionCloseOut = MoveSampleToHumanPlayerZone.endTrajectory().fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();

        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = MoveSampleToHumanPlayerZone.build();
        } else if (startPosition == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }

        Actions.runBlocking(
                new SequentialAction(
                        PlaceSample,
                        hookChamber(),
                        unhookChamber(),
                        trajectoryActionChosen
//                        ,trajectoryActionCloseOut
                )
        );
    }

    public TrajectoryActionBuilder gameLoopSetup(StartingPosition sp, PlacementHeight chamberHeight) {
        baseRobot.logger.update("Autonomous phase", "Placing initial specimen on chamber");
        TrajectoryActionBuilder placingTrajectory = getPlacingTrajectory(sp, roadRunner.actionBuilder(initialPose));

        if(startingPosition == StartingPosition.RED_RIGHT) {
            goober();
            requestOpModeStop();
        }

        Actions.runBlocking(
                new SequentialAction(
                        placingTrajectory.build(),
                        hookChamber()));
        return placingTrajectory;
    }

    public TrajectoryActionBuilder gameLoop(StartingPosition sp, TrajectoryActionBuilder previousTrajectory,
            PlacementHeight placementHeight) {
        baseRobot.telemetry.addData("Autonomous phase", "Grabbing next specimen");
        baseRobot.telemetry.update();
        previousTrajectory = getNextSpecimen(sp, previousTrajectory);
        baseRobot.logger.update("Autonomous phase", "Placing next specimen");
        switch (placementHeight) {
            case CHAMBER_LOW:
            case CHAMBER_HIGH:
                previousTrajectory = placeNextSpecimenOnChamber(sp, previousTrajectory, placementHeight);
            case BASKET_LOW:
            case BASKET_HIGH:
                previousTrajectory = placeNextSampleInBasket(sp, previousTrajectory, placementHeight);

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
            baseRobot.outtake.claw.forward();
            pause(300);
            baseRobot.outtake.verticalSlide.setPosition(ViperSlide.VerticalPosition.HIGH_RUNG);
            pause(2000);
            baseRobot.outtake.linkage.setPosition(Linkage.Position.PLACE);
            pause(2000);
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
            baseRobot.outtake.claw.backward();
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
            baseRobot.intake.wrist.setPosition(Wrist.Position.HORIZONTAL);
            baseRobot.intake.geckoWheels.intake();
            pause(1500);
            baseRobot.intake.wrist.setPosition(Wrist.Position.VERTICAL);
            baseRobot.intake.geckoWheels.stop();
            return false;
        }
    }

    public Action grabSpecimenFromHP() {
        return new GrabSpecimenFromHumanPlayer();
    }

    public class HangAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            baseRobot.outtake.verticalSlide.setPosition(ViperSlide.VerticalPosition.LOW_BASKET);
            baseRobot.outtake.linkage.setPosition(Linkage.Position.PLACE);
            pause(2000);
            return false;
        }
    }

    public Action hang() {
        return new HangAction();
    }

    public TrajectoryActionBuilder placeNextSpecimenOnChamber(StartingPosition sp,
            TrajectoryActionBuilder previousTrajectory, PlacementHeight placementHeight) {
        TrajectoryActionBuilder placingTrajectory = getPlacingTrajectory(sp, previousTrajectory);

        Actions.runBlocking(
                new SequentialAction(
                        placingTrajectory.build(),
                        hookChamber()));

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

    private TrajectoryActionBuilder getParkingTrajectory(MeetFourAuto.StartingPosition sp,
                                                         TrajectoryActionBuilder previousTrajectory) {
        // Helper method to get parking trajectory based on starting position
        switch (sp) {
            case RED_LEFT:
                return previousTrajectory.endTrajectory().fresh()
                        .strafeToLinearHeading(Settings.Autonomous.FieldPositions.RED_PARK_MIDDLEMAN,
                                Math.toRadians(90))
                        .strafeTo(Settings.Autonomous.FieldPositions.RED_LEFT_BEFORE_PARK_POSE.position)
                        .turn(Math.toRadians(90))
                        .strafeTo(Settings.Autonomous.FieldPositions.RED_LEFT_PARK_POSE.position);
            case RED_RIGHT:
                return previousTrajectory.endTrajectory().fresh()
                        .strafeToLinearHeading(Settings.Autonomous.FieldPositions.RED_PARK_MIDDLEMAN,
                                Math.toRadians(90))
                        .strafeTo(Settings.Autonomous.FieldPositions.RED_RIGHT_BEFORE_PARK_POSE.position)
                        .turn(Math.toRadians(90))
                        .strafeTo(Settings.Autonomous.FieldPositions.RED_RIGHT_PARK_POSE.position);
            case BLUE_LEFT:
                return previousTrajectory.endTrajectory().fresh()
                        .strafeToLinearHeading(Settings.Autonomous.FieldPositions.BLUE_PARK_MIDDLEMAN,
                                Math.toRadians(270))
                        .strafeTo(Settings.Autonomous.FieldPositions.BLUE_LEFT_BEFORE_PARK_POSE.position)
                        .turn(Math.toRadians(90))
                        .strafeTo(Settings.Autonomous.FieldPositions.BLUE_LEFT_PARK_POSE.position);
            case BLUE_RIGHT:
                return previousTrajectory.endTrajectory().fresh()
                        .strafeToLinearHeading(Settings.Autonomous.FieldPositions.BLUE_PARK_MIDDLEMAN,
                                Math.toRadians(270))
                        .strafeTo(Settings.Autonomous.FieldPositions.BLUE_RIGHT_BEFORE_PARK_POSE.position)
                        .turn(Math.toRadians(90))
                        .strafeTo(Settings.Autonomous.FieldPositions.BLUE_RIGHT_PARK_POSE.position);
            default:
                return previousTrajectory;
        }
    }

    private TrajectoryActionBuilder getHPTrajectory(StartingPosition sp, TrajectoryActionBuilder previousTrajectory) {
        // Helper method to get human player trajectory based on starting position
        switch (sp) {
            case RED_LEFT:
            case RED_RIGHT:
                return previousTrajectory.endTrajectory().fresh()
                        .strafeToLinearHeading(Settings.Autonomous.FieldPositions.RED_HP_POSE.position,
                                Settings.Autonomous.FieldPositions.RED_HP_POSE.heading);
            case BLUE_LEFT:
            case BLUE_RIGHT:
                return previousTrajectory.endTrajectory().fresh()
                        .strafeToLinearHeading(Settings.Autonomous.FieldPositions.BLUE_HP_POSE.position,
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
                return previousTrajectory.endTrajectory().fresh()
                        .lineToY(initialPose.position.y + 10)
                        .strafeToLinearHeading(Settings.Autonomous.FieldPositions.RED_LEFT_CHAMBER_POSE.position,
                                Settings.Autonomous.FieldPositions.RED_LEFT_CHAMBER_POSE.heading);
            case RED_RIGHT:
                return previousTrajectory.endTrajectory().fresh()
                        .lineToY(initialPose.position.y + 10)
                        .strafeToLinearHeading(Settings.Autonomous.FieldPositions.RED_RIGHT_CHAMBER_POSE.position,
                                Settings.Autonomous.FieldPositions.RED_RIGHT_CHAMBER_POSE.heading);
            case BLUE_LEFT:
                return previousTrajectory.endTrajectory().fresh()
                        .lineToY(initialPose.position.y - 10)
                        .strafeToLinearHeading(Settings.Autonomous.FieldPositions.BLUE_LEFT_CHAMBER_POSE.position,
                                Settings.Autonomous.FieldPositions.BLUE_LEFT_CHAMBER_POSE.heading);
            case BLUE_RIGHT:
                return previousTrajectory.endTrajectory().fresh()
                        .lineToY(initialPose.position.y + 10)
                        .strafeToLinearHeading(Settings.Autonomous.FieldPositions.BLUE_RIGHT_CHAMBER_POSE.position,
                                Settings.Autonomous.FieldPositions.BLUE_RIGHT_CHAMBER_POSE.heading);
            default:
                return previousTrajectory.endTrajectory().fresh();
        }
    }

    private TrajectoryActionBuilder getUnhookTrajectory(StartingPosition sp,
                                                         TrajectoryActionBuilder previousTrajectory) {
        // Helper method to get placing trajectory based on starting position
        switch (sp) {
            case RED_LEFT:
                return previousTrajectory.endTrajectory().fresh()
                        .lineToY(Settings.Autonomous.FieldPositions.RED_LEFT_CHAMBER_POSE.position.y - 5);
            case RED_RIGHT:
                return previousTrajectory.endTrajectory().fresh()
                        .lineToY(Settings.Autonomous.FieldPositions.RED_RIGHT_CHAMBER_POSE.position.y - 5);
            case BLUE_LEFT:
                return previousTrajectory.endTrajectory().fresh()
                        .lineToY(Settings.Autonomous.FieldPositions.BLUE_LEFT_CHAMBER_POSE.position.y + 5);
            case BLUE_RIGHT:
                return previousTrajectory.endTrajectory().fresh()
                        .lineToY(Settings.Autonomous.FieldPositions.BLUE_RIGHT_CHAMBER_POSE.position.y + 5);

            default:
                return previousTrajectory.endTrajectory().fresh();
        }
    }

    private TrajectoryActionBuilder getBasketTrajectory(StartingPosition sp,
            TrajectoryActionBuilder previousTrajectory) {
        switch (sp) {
            case RED_LEFT:
            case RED_RIGHT:
                return previousTrajectory.endTrajectory().fresh()
                        .strafeToLinearHeading(Settings.Autonomous.FieldPositions.RED_BASKET_POSE.position,
                                Settings.Autonomous.FieldPositions.RED_BASKET_POSE.heading);
            case BLUE_LEFT:
            case BLUE_RIGHT:
                return previousTrajectory.endTrajectory().fresh()
                        .strafeToLinearHeading(Settings.Autonomous.FieldPositions.BLUE_BASKET_POSE.position,
                                Settings.Autonomous.FieldPositions.BLUE_BASKET_POSE.heading);
            default:
                return previousTrajectory.endTrajectory().fresh();
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