package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.Extensor;
import org.firstinspires.ftc.teamcode.systems.Logger;
import org.firstinspires.ftc.teamcode.systems.SampleFinder;
import org.firstinspires.ftc.teamcode.mechanisms.submechanisms.Wrist;

/** @noinspection FieldCanBeLocal, unused */
public class MainAuto {
    private final BaseRobot baseRobot;
    private final SampleFinder sampleFinder;
    private final HardwareMap hardwareMap;
    private final Logger logger;

    public MainAuto(BaseRobot base, String color) {
        this.baseRobot = base;
        this.hardwareMap = baseRobot.hardwareMap;
        this.sampleFinder = new SampleFinder(baseRobot, color);
        this.logger = baseRobot.logger;
    }

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

    private void getNextSpecimen() {
        // we are in the center by the rungs
        // move to the human player
        baseRobot.odometry.moveCounts("backward", 50); // TODO tune
        baseRobot.odometry.moveCounts("right", 50); // TODO tune
        baseRobot.odometry.moveCounts("tright", 50); // TODO tune
        getSpecimenFromHumanPlayer();
    }

    private void getSpecimenFromHumanPlayer() {
        baseRobot.arm.wrist.setPosition(Wrist.Position.NEUTRAL);
        baseRobot.arm.claw.open();
        baseRobot.arm.extensor.setPosition(Extensor.Position.PICKUP);
        pause(500);
        baseRobot.arm.claw.close();
        pause(250);
        baseRobot.arm.extensor.setPosition(Extensor.Position.HOVER);
    }

    private void dropSample() {
        baseRobot.arm.wrist.setPosition(Wrist.Position.HORIZONTAL);
        baseRobot.arm.extensor.ground();
        pause(1000);
        baseRobot.arm.claw.setRightServo(true);
        pause(500);
        baseRobot.arm.extensor.setPosition(Extensor.Position.HOVER);
        baseRobot.arm.wrist.setPosition(Wrist.Position.NEUTRAL);
        baseRobot.arm.claw.setRightServo(false);
        pause(1000);
    }

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

    public void placeNextSpecimenOnChamber(ChamberHeight chamberHeight) {
        baseRobot.odometry.update(); // Update robot's current position

        baseRobot.odometry.moveCounts("left", 50); // TODO tune
        baseRobot.odometry.moveCounts("forward", 100); // TODO tune
        placeSpecimen(chamberHeight);
    }

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

    public void park(String mode) {
        baseRobot.odometry.moveCounts("backward", 100); // TODO tune
        baseRobot.odometry.moveCounts("right", 100); // TODO tune
    }

    public void immediatelyPark(String mode) {
        switch (mode) {
            case "red right":
            case "blue right":
                baseRobot.odometry.moveCounts("right", 50); // TODO tune
            case "red left":
            case "blue left":
                baseRobot.odometry.moveCounts("right", 80); // TODO tune
        }
    }

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

    private void pause(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public enum ChamberHeight {
        LOW,
        HIGH
    }
}
