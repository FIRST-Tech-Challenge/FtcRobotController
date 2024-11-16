package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.Extensor;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.systems.Logger;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.Wrist;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Action;

/** @noinspection FieldCanBeLocal, unused */
public class MainAuto {
    private final BaseRobot baseRobot;
    private final HardwareMap hardwareMap;
    private final Logger logger;
    private final MecanumDrive drive;

    private final String allianceColor;
    private final Vector2d scoringPosition;
    private final Vector2d humanPlayerPosition;
    private final Vector2d parkingPosition;

    /**
     * Creates a new autonomous controller instance
     * 
     * @param base     Base robot instance to control
     * @param color    Selected alliance color
     * @param position Starting position ("left" or "right")
     */
    public MainAuto(BaseRobot base, String color, String position) {
        this.baseRobot = base;
        this.hardwareMap = baseRobot.hardwareMap;
        this.logger = baseRobot.logger;
        this.allianceColor = color;

        // Set positions based on alliance color
        if (color.equalsIgnoreCase("red")) {
            scoringPosition = Settings.Autonomous.FieldPositions.RED_SCORING_POSITION;
            humanPlayerPosition = Settings.Autonomous.FieldPositions.RED_HUMAN_PLAYER;
            parkingPosition = Settings.Autonomous.FieldPositions.RED_PARKING;
        } else {
            scoringPosition = Settings.Autonomous.FieldPositions.BLUE_SCORING_POSITION;
            humanPlayerPosition = Settings.Autonomous.FieldPositions.BLUE_HUMAN_PLAYER;
            parkingPosition = Settings.Autonomous.FieldPositions.BLUE_PARKING;
        }

        // Initialize drive with correct starting position
        Pose2d initialPose;
        switch (color.toLowerCase() + " " + position.toLowerCase()) {
            case "red right":
                initialPose = new Pose2d(Settings.Autonomous.FieldPositions.RED_RIGHT_START, 0);
                break;
            case "red left":
                initialPose = new Pose2d(Settings.Autonomous.FieldPositions.RED_LEFT_START, 0);
                break;
            case "blue right":
                initialPose = new Pose2d(Settings.Autonomous.FieldPositions.BLUE_RIGHT_START, Math.PI);
                break;
            case "blue left":
                initialPose = new Pose2d(Settings.Autonomous.FieldPositions.BLUE_LEFT_START, Math.PI);
                break;
            default:
                initialPose = new Pose2d(0, 0, 0);
        }
        this.drive = new MecanumDrive(hardwareMap, initialPose);
    }

    /**
     * Executes the main autonomous routine
     * 
     * @param mode Selected autonomous mode (e.g., "red left", "blue right")
     */
    public void run(String mode) {

        if (Settings.Deploy.SKIP_AUTONOMOUS) {
            baseRobot.logger.update("Autonomous phase", "Skipping due to deploy flag");
            immediatelyPark(mode);
            return;
        }

        baseRobot.logger.update("Autonomous phase", "Placing initial specimen on chamber");
        placeOnChamber(mode, ChamberHeight.HIGH);
        while (30 - baseRobot.parentOp.getRuntime() > (Settings.ms_needed_to_park / 1000)) {
            baseRobot.logger.update("Autonomous phase", "Grabbing next specimen");
            getNextSpecimen();
            baseRobot.logger.update("Autonomous phase", "Placing next specimen");
            placeNextSpecimenOnChamber(ChamberHeight.HIGH);
        }
        baseRobot.logger.update("Autonomous phase", "Parking");
        park(mode);
        baseRobot.logger.update("Autonomous phase", "VICTORY!!!");
        if (Settings.Deploy.VICTORY) {
            victory();
        }
    }

    /**
     * Retrieves the next specimen from the collection area
     * Sequence:
     * 1. Move to collection position
     * 2. Position arm for pickup
     * 3. Grab specimen
     * 4. Return to safe position
     */
    private void getNextSpecimen() {
        Action trajectory = drive.actionBuilder(drive.pose)
                .lineToX(humanPlayerPosition.x)
                .lineToY(humanPlayerPosition.y)
                .build();

        Actions.runBlocking(trajectory);
        getSpecimenFromHumanPlayer();
    }

    /**
     * Coordinates with human player to receive specimen
     * Sequence:
     * 1. Position arm in neutral position
     * 2. Open claw
     * 3. Move to pickup position
     * 4. Close claw on specimen
     * 5. Return to hover position
     */
    private void getSpecimenFromHumanPlayer() {
        baseRobot.arm.wrist.setPosition(Wrist.Position.VERTICAL);
        baseRobot.arm.claw.open();
        baseRobot.arm.extensor.setPosition(Extensor.Position.PICKUP);
        pause(500);
        baseRobot.arm.wrist.setPosition(Wrist.Position.HORIZONTAL);
        pause(100);
        baseRobot.arm.claw.close();
        pause(250);
        baseRobot.arm.extensor.setPosition(Extensor.Position.HOVER);
    }

    /**
     * Places specimen on the scoring chamber
     * 
     * @param mode          Current autonomous mode
     * @param chamberHeight Target chamber height (HIGH/LOW)
     */
    public void placeOnChamber(String mode, ChamberHeight chamberHeight) {
        drive.updatePoseEstimate();

        Action trajectory = drive.actionBuilder(drive.pose)
                .lineToX(scoringPosition.x)
                .lineToY(scoringPosition.y)
                .build();

        Actions.runBlocking(trajectory);
        placeSpecimen(chamberHeight);
    }

    /**
     * Places a specimen on the chamber during cycling sequence
     * 
     * @param chamberHeight Target chamber height (HIGH/LOW)
     */
    public void placeNextSpecimenOnChamber(ChamberHeight chamberHeight) {
        drive.updatePoseEstimate();

        Action trajectory = drive.actionBuilder(drive.pose)
                .lineToY(drive.pose.position.y - 50) // left 50
                .lineToX(drive.pose.position.x + 100) // forward 100
                .build();

        Actions.runBlocking(trajectory);
        placeSpecimen(chamberHeight);
    }

    /**
     * Executes the specimen placement sequence
     * 
     * @param chamberHeight Target chamber height (HIGH/LOW)
     */
    private void placeSpecimen(ChamberHeight chamberHeight) {
        Extensor.Position placeHeight;
        if (chamberHeight == ChamberHeight.HIGH) {
            placeHeight = Extensor.Position.HIGH_RUNG;
        } else {
            placeHeight = Extensor.Position.LOW_RUNG;
        }

        baseRobot.arm.extensor.setPosition(placeHeight);
        baseRobot.arm.wrist.setPosition(Wrist.Position.CHAMBER);
        baseRobot.odometry.moveCounts("forward", 60, 0.1);
        pause(2500);
        baseRobot.arm.claw.open();
        pause(1000);
        baseRobot.odometry.moveCounts("backward", 10, 0.1);
        baseRobot.arm.wrist.setPosition(Wrist.Position.VERTICAL);
        baseRobot.arm.extensor.setPosition(Extensor.Position.HOVER);
        pause(2000);
    }

    /**
     * Moves robot to parking position based on selected mode
     * 
     * @param mode Current autonomous mode
     */
    public void park(String mode) {
        drive.updatePoseEstimate();

        Action trajectory = drive.actionBuilder(drive.pose)
                .lineToX(parkingPosition.x)
                .lineToY(parkingPosition.y)
                .build();

        Actions.runBlocking(trajectory);
    }

    /**
     * Emergency parking routine - moves directly to parking position
     * 
     * @param mode Current autonomous mode
     */
    public void immediatelyPark(String mode) {
        switch (mode.toLowerCase()) {
            // ! TODO this all sucks
            case "red right":
            case "blue right":
                baseRobot.mecanumDrive(0, -0.5, 0);
                pause(2000);
                baseRobot.mecanumDrive(0, 0, 0);
                break;
            case "red left":
            case "blue left":
                baseRobot.mecanumDrive(0, 0.5, 0);
                pause(2000);
                baseRobot.mecanumDrive(0, 0, 0);
                break;
        }
        baseRobot.telemetry.addData("Yes, we are parking as ", mode);
    }

    /**
     * Executes victory celebration sequence if enabled
     * Performs a series of movements and claw operations
     */
    public void victory() {
        // do a lil dance
        baseRobot.arm.claw.open();
        pause(500);
        baseRobot.arm.claw.close();
        pause(500);
        baseRobot.odometry.moveCounts("tright", 20, 0.2);
        baseRobot.odometry.moveCounts("tleft", 40, 0.2);
        baseRobot.odometry.moveCounts("tright", 20, 0.2);
        baseRobot.arm.claw.open();
        pause(500);
        baseRobot.arm.claw.close();
        pause(500);
    }

    /**
     * Utility method to pause execution
     * 
     * @param ms Milliseconds to pause
     */
    private void pause(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
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
}
