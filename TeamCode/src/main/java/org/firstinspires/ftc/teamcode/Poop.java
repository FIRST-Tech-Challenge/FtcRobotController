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

@Autonomous(name = "poop", group = "Autonomous")
public class Poop extends LinearOpMode {

    private BaseRobot baseRobot;

    private MecanumDrive roadRunner;

    private Pose2d initialPose;

    public AdaptiveCalibration adaptiveCalibration;

    @Override
    public void runOpMode() {
        adaptiveCalibration = AdaptiveCalibration.getInstance();
        DynamicInput dynamicInput = new DynamicInput(gamepad1, gamepad2,
                Settings.DEFAULT_PROFILE, Settings.DEFAULT_PROFILE);
        baseRobot = new BaseRobot(hardwareMap, dynamicInput, this, telemetry);

        initialPose = Settings.Autonomous.FieldPositions.RIGHT_INITIAL_POSE;


        roadRunner = new MecanumDrive(hardwareMap, initialPose);
        adaptiveCalibration.initialize(roadRunner);
        waitForStart();

        run();
    }

    public void run() {
        baseRobot.outtake.linkage.setPosition(Linkage.Position.PLACE_BACKWARD);
        baseRobot.outtake.verticalSlide.setPosition(ViperSlide.VerticalPosition.TRANSFER);
        TrajectoryActionBuilder previousChamberTrajectory = gameLoopSetup();
        while (30 - baseRobot.parentOp.getRuntime() > (Settings.ms_needed_to_park / 1000)) {
            adaptiveCalibration.calibrateRuntime(new AdaptiveCalibration.RuntimeCalibrationPayload(), roadRunner);
            previousChamberTrajectory = gameLoop(previousChamberTrajectory);
        }
        gameLoopEnd(previousChamberTrajectory);
    }

    public TrajectoryActionBuilder gameLoopSetup() {
        baseRobot.logger.update("Autonomous phase", "Placing initial specimen on chamber");
        TrajectoryActionBuilder placingTrajectory = getPlacingTrajectory(roadRunner.actionBuilder(initialPose));
        TrajectoryActionBuilder unhookTrajectory = getUnhookTrajectory(placingTrajectory);

        Actions.runBlocking(
                new SequentialAction(
                        placingTrajectory.build(),
                        hookChamber(),
                        unhookChamber()));

        return placingTrajectory;
    }

    public TrajectoryActionBuilder gameLoop(TrajectoryActionBuilder previousTrajectory) {
        baseRobot.telemetry.addData("Autonomous phase", "Grabbing next specimen");
        baseRobot.telemetry.update();
        previousTrajectory = getNextSpecimen(previousTrajectory);
        baseRobot.logger.update("Autonomous phase", "Placing next specimen");
        previousTrajectory = placeNextSpecimenOnChamber(previousTrajectory);
        return previousTrajectory;
    }

    public TrajectoryActionBuilder gameLoopEnd(TrajectoryActionBuilder previousPose) {
        TrajectoryActionBuilder parkingTrajectory = getParkingTrajectory(previousPose);

        Actions.runBlocking(
                new SequentialAction(
                        parkingTrajectory.build()));

        return parkingTrajectory;
    }


    public class HookChamber implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            baseRobot.outtake.claw.close();
            baseRobot.outtake.verticalSlide.setPosition(ViperSlide.VerticalPosition.HIGH_RUNG);
            pause(500);
            baseRobot.outtake.linkage.setPosition(Linkage.Position.PLACE_FORWARD);
            pause(700);
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
            baseRobot.outtake.linkage.setPosition(Linkage.Position.PLACE_BACKWARD);
            pause(2000);
            return false;
        }
    }

    public Action hang() {
        return new HangAction();
    }

    public TrajectoryActionBuilder placeNextSpecimenOnChamber(TrajectoryActionBuilder previousTrajectory) {
        TrajectoryActionBuilder placingTrajectory = getPlacingTrajectory(previousTrajectory);

        Actions.runBlocking(
                new SequentialAction(
                        placingTrajectory.build(),
                        hookChamber()));

        return placingTrajectory;
    }

    public TrajectoryActionBuilder getNextSpecimen(TrajectoryActionBuilder previousTrajectory) {
        TrajectoryActionBuilder hpTrajectory = getHPTrajectory(previousTrajectory);
        Actions.runBlocking(
                new SequentialAction(
                        hpTrajectory.build(),
                        grabSpecimenFromHP()));
        return hpTrajectory;
    }

    private TrajectoryActionBuilder getParkingTrajectory(TrajectoryActionBuilder previousTrajectory) {
        return previousTrajectory.endTrajectory().fresh()
                .strafeToLinearHeading(Settings.Autonomous.FieldPositions.PARK_MIDDLEMAN,
                        Math.toRadians(90))
                .strafeTo(Settings.Autonomous.FieldPositions.RIGHT_BEFORE_PARK_POSE.position)
                .turn(Math.toRadians(90))
                .strafeTo(Settings.Autonomous.FieldPositions.RIGHT_PARK_POSE.position);
    }

    private TrajectoryActionBuilder getHPTrajectory(TrajectoryActionBuilder previousTrajectory) {
        return previousTrajectory.endTrajectory().fresh()
                .strafeToLinearHeading(Settings.Autonomous.FieldPositions.HP_POSE.position,
                        Settings.Autonomous.FieldPositions.HP_POSE.heading);
    }

    private TrajectoryActionBuilder getPlacingTrajectory(TrajectoryActionBuilder previousTrajectory) {
        return previousTrajectory.endTrajectory().fresh()
                .lineToY(initialPose.position.y + 10)
                .strafeToLinearHeading(Settings.Autonomous.FieldPositions.RIGHT_CHAMBER_POSE.position,
                        Settings.Autonomous.FieldPositions.RIGHT_CHAMBER_POSE.heading);
    }

    private TrajectoryActionBuilder getUnhookTrajectory(TrajectoryActionBuilder previousTrajectory) {
                return previousTrajectory.endTrajectory().fresh()
                        .lineToY(Settings.Autonomous.FieldPositions.RIGHT_CHAMBER_POSE.position.y - 2);
    }

    private void pause(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}