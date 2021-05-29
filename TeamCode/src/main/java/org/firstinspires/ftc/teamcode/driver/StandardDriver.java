package org.firstinspires.ftc.teamcode.driver;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.parts.RobotPartSettings;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.drivetrains.DriveTrain;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.movement.Movement;
import org.firstinspires.ftc.teamcode.robot.parts.drivetrain.movement.Turn;

import java.util.ArrayList;

/**
 * Drive code for a person controlling the robot with StandardDrive
 * @author 22jmiller
 */
public class  StandardDriver extends Driver {
    private boolean aPressing = false;

    private boolean useTelemetry;

    private StandardDriveMode standardDriveMode;

    /**
     *
     * @param useTelemetry Pretty straight forward, let this class use telemetry, output stuff?
     * @param standardDriveMode How do you want to drive the car?
     * @param robotPartSettings What type of wheel do you have?
     * @param gamepad1 Gamepad 1
     * @param gamepad2 Gamepad 2
     * @param telemetry Telemetry
     * @param hardwareMap Hardware Map
     */
    public StandardDriver(boolean useTelemetry, StandardDriveMode standardDriveMode, ArrayList<RobotPartSettings> robotPartSettings, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, HardwareMap hardwareMap) {
        this (useTelemetry, standardDriveMode, new Robot(robotPartSettings, gamepad1, gamepad2,telemetry,hardwareMap));
    }

    public StandardDriver(boolean useTelemetry, StandardDriveMode standardDriveMode, Robot robot) {
        super(robot);
        this.useTelemetry = useTelemetry;
        this.standardDriveMode = standardDriveMode;
    }

    /**
     * Loop of the robot. Hook this up to the loop method in the OP Mode
     */
    public void loop() {
        // Change Mode
        if (robot.gamepad1.a) { // a pressed
            if (!aPressing) {
                aPressed();
                aPressing = true;
            }
        } else {
            aPressing = false;
        }

        // Move / Drive
        switch (standardDriveMode) {
            case TWO_STICK:
                ((DriveTrain)robot.getRobotPart(DriveTrain.class)).move(Movement.FORWARDS, -robot.gamepad1.left_stick_y);
                ((DriveTrain)robot.getRobotPart(DriveTrain.class)).turn(Turn.CLOCKWISE, robot.gamepad1.right_stick_x);
                break;
            case ARCADE:
                ((DriveTrain)robot.getRobotPart(DriveTrain.class)).move(Movement.FORWARDS, -robot.gamepad1.left_stick_y);
                ((DriveTrain)robot.getRobotPart(DriveTrain.class)).turn(Turn.CLOCKWISE, robot.gamepad1.left_stick_x);
                break;
            case GTA:
                ((DriveTrain)robot.getRobotPart(DriveTrain.class)).move(Movement.BACKWARDS, robot.gamepad1.left_trigger); // Backwards
                ((DriveTrain)robot.getRobotPart(DriveTrain.class)).move(Movement.FORWARDS, robot.gamepad1.right_trigger); // Forwards
                ((DriveTrain)robot.getRobotPart(DriveTrain.class)).turn(Turn.CLOCKWISE, robot.gamepad1.left_stick_x); // Turning
                break;
            case FORZA:
                if (robot.gamepad1.y) ((DriveTrain)robot.getRobotPart(DriveTrain.class)).move(Movement.BACKWARDS, robot.gamepad1.right_trigger); // Backwards
                else ((DriveTrain)robot.getRobotPart(DriveTrain.class)).move(Movement.FORWARDS, robot.gamepad1.right_trigger); // Forwards
                ((DriveTrain)robot.getRobotPart(DriveTrain.class)).turn(Turn.CLOCKWISE, robot.gamepad1.left_stick_x); // Turning
                break;
            default:
                stop();
                break;
        }

    }

    private void aPressed() {
        standardDriveMode = standardDriveMode.next();
    }
}
