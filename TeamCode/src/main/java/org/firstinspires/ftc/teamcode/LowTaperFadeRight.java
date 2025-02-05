package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Settings.Autonomous.FieldPositions.SAFE_CHAMBER_VEC;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.Linkage;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.ViperSlide;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.Wrist;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.systems.DynamicInput;

import java.util.Arrays;

@Autonomous(name = "Low Taper Fade Right", group = "Autonomous")
public class LowTaperFadeRight extends LinearOpMode {
    private BaseRobot baseRobot;
    private MecanumDrive roadRunner;
    private Pose2d initialPose;
    public AdaptiveCalibration adaptiveCalibration;

    public static VelConstraint speedyVel = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(70),
            new AngularVelConstraint(Math.PI)
    ));
    public static AccelConstraint speedyAccel = new ProfileAccelConstraint(-70, 60);

    @Override
    public void runOpMode() {
        adaptiveCalibration = AdaptiveCalibration.getInstance();
        DynamicInput dynamicInput = new DynamicInput(gamepad1, gamepad2,
                Settings.DEFAULT_PROFILE, Settings.DEFAULT_PROFILE);
        baseRobot = new BaseRobot(hardwareMap, dynamicInput, this, telemetry);

        initialPose = Settings.Autonomous.FieldPositions.RIGHT_INITIAL_POSE;
        roadRunner = new MecanumDrive(hardwareMap, initialPose);
        adaptiveCalibration.initialize(roadRunner);

        telemetry.addData("Status", "Ready to start on the right side");
        telemetry.update();
        waitForStart();

        run();
    }

    public void run() {
        baseRobot.intake.wrist.setPosition(Wrist.Position.VERTICAL);
        TrajectoryActionBuilder previousChamberTrajectory = gameLoopSetup(PlacementHeight.CHAMBER_HIGH);
        int phase = 0;
        while (phase < 2) {
            phase++;
            adaptiveCalibration.calibrateRuntime(new AdaptiveCalibration.RuntimeCalibrationPayload(), roadRunner);
            previousChamberTrajectory = placeLoop(previousChamberTrajectory, PlacementHeight.CHAMBER_HIGH, phase);
        }
        baseRobot.logger.update("Autonomous phase", "Parking");
        gameLoopEnd(previousChamberTrajectory);
        baseRobot.logger.update("Autonomous phase", "Victory is ours");
    }

    public TrajectoryActionBuilder gameLoopSetup(PlacementHeight chamberHeight) {
        baseRobot.logger.update("Autonomous phase", "Placing initial specimen on chamber");
        TrajectoryActionBuilder placingTrajectory = getPlacingTrajectory(roadRunner.actionBuilder(initialPose), 0);
        TrajectoryActionBuilder sampleTrajectory = pushSamples(placingTrajectory);
        TrajectoryActionBuilder placingTrajectory2 = getPlacingTrajectory(sampleTrajectory, 0);
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

        return placingTrajectory2;
    }

    public TrajectoryActionBuilder placeLoop(TrajectoryActionBuilder previousTrajectory,
                                             PlacementHeight placementHeight, int blocksScored) {
        baseRobot.telemetry.addData("Autonomous phase", "Grabbing next specimen");
        baseRobot.telemetry.update();
        baseRobot.outtake.verticalSlide.setPosition(ViperSlide.VerticalPosition.TRANSFER);
        baseRobot.outtake.linkage.setPosition(Linkage.Position.PLACE_BACKWARD);
        baseRobot.logger.update("Autonomous phase", "Getting next specimen");
        previousTrajectory = getNextSpecimen(previousTrajectory);
        previousTrajectory = placeNextSpecimenOnChamber(previousTrajectory, placementHeight, blocksScored);
        return previousTrajectory;
    }

    public TrajectoryActionBuilder gameLoopEnd(TrajectoryActionBuilder previousPose) {
        TrajectoryActionBuilder parkingTrajectory = getParkingTrajectory(previousPose);

        Actions.runBlocking(
                new SequentialAction(
                        parkingTrajectory.build()));

        return parkingTrajectory;
    }

    public TrajectoryActionBuilder placeNextSpecimenOnChamber(TrajectoryActionBuilder previousTrajectory,
                                                              PlacementHeight placementHeight, int specimensScored) {
        TrajectoryActionBuilder placingTrajectory = getPlacingTrajectory(previousTrajectory, specimensScored);
        Actions.runBlocking(
                new SequentialAction(
                        placingTrajectory.build(),
                        hookChamber(),
                        unhookChamber()
                ));
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
                .strafeTo(Settings.Autonomous.FieldPositions.RIGHT_PARK_POSE.position);
    }

    private TrajectoryActionBuilder getHPTrajectory(TrajectoryActionBuilder previousTrajectory) {
        return previousTrajectory.endTrajectory().fresh()
                .strafeTo(new Vector2d(Settings.Autonomous.FieldPositions.HP_POSE.position.x, Settings.Autonomous.FieldPositions.HP_POSE.position.y), speedyVel, speedyAccel)
                .waitSeconds(.1)
                .lineToY(Settings.Autonomous.FieldPositions.HP_POSE.position.y - 10, speedyVel, speedyAccel)
                .waitSeconds(0.5);
    }

    private TrajectoryActionBuilder getPlacingTrajectory(TrajectoryActionBuilder previousTrajectory, int specimensScored) {
        return previousTrajectory.endTrajectory().fresh()
                .strafeTo(SAFE_CHAMBER_VEC)
                .strafeTo(new Vector2d(Settings.Autonomous.FieldPositions.RIGHT_CHAMBER_POSE.position.x, Settings.Autonomous.FieldPositions.RIGHT_CHAMBER_POSE.position.y),
                        speedyVel, speedyAccel);
    }

    private TrajectoryActionBuilder pushSamples(TrajectoryActionBuilder previousTrajectory) {
        return previousTrajectory.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .strafeTo(Settings.Autonomous.FieldPositions.SAMPLE_MIDDLEMAN)
                .splineToLinearHeading(new Pose2d(Settings.Autonomous.FieldPositions.FIRST_PRESET_SAMPLE_POSE.position,
                        Settings.Autonomous.FieldPositions.FIRST_PRESET_SAMPLE_POSE.heading), Math.toRadians(270))
                .lineToY(Settings.Autonomous.FieldPositions.FIRST_PRESET_SAMPLE_POSE.position.y - 50)
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(Settings.Autonomous.FieldPositions.SECOND_PRESET_SAMPLE_POSE.position,
                        Settings.Autonomous.FieldPositions.SECOND_PRESET_SAMPLE_POSE.heading), Math.toRadians(270))
                .lineToY(Settings.Autonomous.FieldPositions.SECOND_PRESET_SAMPLE_POSE.position.y - 58, speedyVel, speedyAccel);
    }

    public Action hookChamber() {
        return new HookChamber();
    }

    public Action unhookChamber() {
        return new UnhookChamber();
    }

    public Action grabSpecimenFromHP() {
        return new GrabSpecimenFromHumanPlayer();
    }

    public class HookChamber implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            baseRobot.outtake.linkage.setPosition(Linkage.Position.PLACE_BACKWARD);
            baseRobot.outtake.verticalSlide.setPosition(ViperSlide.VerticalPosition.HIGH_RUNG.getValue() + 650);
            pause(1000);
            baseRobot.outtake.claw.open();
            return false;
        }
    }

    public class UnhookChamber implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            baseRobot.outtake.verticalSlide.setPosition(ViperSlide.VerticalPosition.TRANSFER);
            baseRobot.outtake.linkage.setPosition(Linkage.Position.PLACE_BACKWARD);
            return false;
        }
    }

    public class GrabSpecimenFromHumanPlayer implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            baseRobot.outtake.claw.close();
            pause(500);
            baseRobot.outtake.linkage.setPosition(Linkage.Position.PLACE_FORWARD);
            baseRobot.outtake.verticalSlide.setPosition(Settings.Hardware.VerticalSlide.HIGH_RUNG_PREP_AUTO);
            pause(500);
            return false;
        }
    }

    private void pause(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public enum PlacementHeight {
        CHAMBER_LOW,
        CHAMBER_HIGH
    }
}