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
        baseRobot.arm.wrist.setPosition(Wrist.Position.TRANSIT);
        baseRobot.logger.update("Autonomous phase", "Placing sample onto submersible");
        placeOnSubmersible(mode);
        baseRobot.logger.update("Autonomous phase", "Parking");
        park(mode);
        baseRobot.logger.update("Autonomous phase", "Complete");
        if (Settings.Deploy.VICTORY) {
            victory();
        }
    }

    private void dropSample() {
        baseRobot.arm.wrist.setPosition(Wrist.Position.HORIZONTAL);
        baseRobot.arm.extensor.ground();
        pause(1000);
        baseRobot.arm.claw.setRightServo(true);
        pause(500);
        baseRobot.arm.extensor.setPosition(Extensor.Position.HOVER);
        baseRobot.arm.wrist.setPosition(Wrist.Position.TRANSIT);
        baseRobot.arm.claw.setRightServo(false);
        pause(1000);
    }


    public void placeOnSubmersible(String mode) {
        baseRobot.odometry.update(); // Update robot's current position

        switch (mode) {
            case "red right":
            case "blue right":
                baseRobot.odometry.moveCounts("left", 50);
                baseRobot.odometry.moveCounts("forward", 100);
                placeSpecimen();
                break;
            case "red left":
            case "blue left":
                baseRobot.odometry.moveCounts("right", 50);
                baseRobot.odometry.moveCounts("forward", 100);
                placeSpecimen();
                break;
        }
    }

    private void placeSpecimen() {
        baseRobot.arm.extender.set(Extensor.Position.LEVEL1);
        baseRobot.arm.wrist.setPosition(Wrist.Position.BOARD);
        baseRobot.odometry.moveCounts("forward", 60, 0.1);
        pause(2500);
        baseRobot.arm.claw.open();
        pause(1000);
        baseRobot.odometry.moveCounts("backward", 10, 0.1);
        baseRobot.arm.wrist.setPosition(Wrist.Position.TRANSIT);
        baseRobot.arm.extender.set(Extensor.Position.HOVER);
        pause(2000);
    }

    public void park(String mode) {
        switch (mode) {
            case "red right":
            case "blue right":
                baseRobot.odometry.moveCounts("backward", 100);
                baseRobot.odometry.moveCounts("right", 100);
                break;
            case "red left":
            case "blue left":
                baseRobot.odometry.moveCounts("backward", 100);
                baseRobot.odometry.moveCounts("left", 100);
                break;
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
}
