package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.Extensor;
import org.firstinspires.ftc.teamcode.systems.Logger;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.Wrist;

/** @noinspection FieldCanBeLocal, unused */
public class MainAuto {
    private final BaseRobot baseRobot;
    private final HardwareMap hardwareMap;
    private final Logger logger;

    /**
     * Creates a new autonomous controller instance
     * 
     * @param base  Base robot instance to control
     * @param color Selected alliance color
     */
    public MainAuto(BaseRobot base, String color) {
        this.baseRobot = base;
        this.hardwareMap = baseRobot.hardwareMap;
        this.logger = baseRobot.logger;
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
        baseRobot.odometry.moveCounts("backward", Settings.Autonomous.Movement.BACKWARD_COUNTS);
        baseRobot.odometry.moveCounts("right", Settings.Autonomous.Movement.STRAFE_COUNTS);
        baseRobot.odometry.moveCounts("tright", Settings.Autonomous.Movement.TURN_COUNTS);
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
        baseRobot.arm.wrist.setPosition(Wrist.Position.NEUTRAL);
        baseRobot.arm.claw.open();
        baseRobot.arm.extensor.setPosition(Extensor.Position.PICKUP);
        pause(500);
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
        baseRobot.odometry.update(); // Update robot's current position

        switch (mode) {
            case "red right":
            case "blue right":
                baseRobot.odometry.moveCounts("left", 50);
                baseRobot.odometry.moveCounts("forward", 100);
                placeSpecimen(chamberHeight);
                break;
            case "red left":
            case "blue left":
                baseRobot.odometry.moveCounts("right", 50);
                baseRobot.odometry.moveCounts("forward", 100);
                placeSpecimen(chamberHeight);
                break;
        }
    }

    /**
     * Places a specimen on the chamber during cycling sequence
     * 
     * @param chamberHeight Target chamber height (HIGH/LOW)
     */
    public void placeNextSpecimenOnChamber(ChamberHeight chamberHeight) {
        baseRobot.odometry.update(); // Update robot's current position

        baseRobot.odometry.moveCounts("left", 50); // TODO tune
        baseRobot.odometry.moveCounts("forward", 100); // TODO tune
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
        baseRobot.arm.wrist.setPosition(Wrist.Position.RUNG);
        baseRobot.odometry.moveCounts("forward", 60, 0.1);
        pause(2500);
        baseRobot.arm.claw.open();
        pause(1000);
        baseRobot.odometry.moveCounts("backward", 10, 0.1);
        baseRobot.arm.wrist.setPosition(Wrist.Position.NEUTRAL);
        baseRobot.arm.extensor.setPosition(Extensor.Position.HOVER);
        pause(2000);
    }

    /**
     * Moves robot to parking position based on selected mode
     * 
     * @param mode Current autonomous mode
     */
    public void park(String mode) {
        baseRobot.odometry.moveCounts("backward", 100); // TODO tune
        baseRobot.odometry.moveCounts("right", 100); // TODO tune
    }

    /**
     * Emergency parking routine - moves directly to parking position
     * 
     * @param mode Current autonomous mode
     */
    public void immediatelyPark(String mode) {
        switch (mode) {
            case "red right":
            case "blue right":
                baseRobot.odometry.moveCounts("right", 50);
                break;
            case "red left":
            case "blue left":
                baseRobot.odometry.moveCounts("right", 80);
                break;
        }
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
