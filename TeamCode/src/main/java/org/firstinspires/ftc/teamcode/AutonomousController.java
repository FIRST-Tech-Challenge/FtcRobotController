package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.systems.Logger;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public abstract class AutonomousController {
    protected final BaseRobot baseRobot;
    protected final HardwareMap hardwareMap;
    protected final Logger logger;
    protected final MecanumDrive drive;

    protected final String allianceColor;
    protected final Vector2d scoringPosition;
    protected final Vector2d humanPlayerPosition;
    protected final Vector2d parkingPosition;

    public AutonomousController(BaseRobot base, String color, String position) {
        this.baseRobot = base;
        this.hardwareMap = base.hardwareMap;
        this.logger = base.logger;
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

        this.drive = initializeDrive(color, position);
    }

    protected abstract MecanumDrive initializeDrive(String color, String position);

    public abstract void run(String mode);

    public abstract void placeOnChamber(String mode, ChamberHeight chamberHeight);

    public abstract void placeNextSpecimenOnChamber(ChamberHeight chamberHeight);

    // Common utility methods and enums can be defined here
    protected void pause(long ms) {
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
}