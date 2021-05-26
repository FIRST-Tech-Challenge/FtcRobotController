package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.core.Command;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftTele;

public class IntakeCommands extends Command {

    public IntakeSubsystem intake;
    public UpliftTele opMode;
    UpliftRobot robot;
    public boolean lifterButtonPressed;
    public boolean stickButtonPressed;

    public IntakeCommands(UpliftTele opMode, UpliftRobot robot, IntakeSubsystem intakeSubsystem) {
        super(opMode, intakeSubsystem);
        this.intake = intakeSubsystem;
        this.opMode = opMode;
        this.robot = robot;
    }

    @Override
    public void init() {
        intake.initStick();
    }

    @Override
    public void start() {
        intake.dropRoller();
        intake.dropSweeper();
        intake.dropStick();
    }

    @Override
    public void loop() {

        intake.setIntakePower(Range.clip(opMode.gamepad2.left_stick_y, -1, 1));

        // Toggle Button for stick
        if(opMode.gamepad1.dpad_right) {
            if(!stickButtonPressed) {
                robot.stickToggle = !robot.stickToggle;
                stickButtonPressed = true;
            }
        } else {
            stickButtonPressed = false;
        }

        // Toggle Button for Roller to Move up and Down
        if(opMode.gamepad2.left_bumper) {
            if (!lifterButtonPressed) {
                robot.intakeToggle = !robot.intakeToggle;
                lifterButtonPressed = true;
            }
        } else {
            lifterButtonPressed = false;
        }

        if(robot.driverCancel || robot.operatorCancel) {
            robot.forceLiftRoller = false;
        }

        if(intake.getPower() < 0) {
            intake.dropRoller();
        } else if(robot.forceLiftRoller) {
            intake.initRoller();
        } else if(robot.intakeToggle) {
            intake.liftRoller();
        } else {
            intake.dropRoller();
        }

        if(intake.getPower() < 0) {
            intake.sweeperOn();
        } else {
            intake.sweeperOff();
        }

        if(robot.stickToggle || robot.shootingState == UpliftRobot.ShootingState.PREPARING_POWERSHOT) {
            robot.setStickState(UpliftRobot.StickState.POWERSHOT_POSITION);
            intake.stick.setPosition(0.35);
        } else if(robot.shootingState == UpliftRobot.ShootingState.PREPARING_HIGHGOAL || robot.shootingState == UpliftRobot.ShootingState.SHOOTING_HIGHGOAL && robot.shotCount < 1) {
            robot.setStickState(UpliftRobot.StickState.UP);
            intake.raiseStick();
        } else if(robot.shootingState == UpliftRobot.ShootingState.DONE_SHOOTING || robot.stickState == UpliftRobot.StickState.DOWN) {
            intake.dropStick();
        } else {
            intake.dropStick();
        }
    }

    @Override
    public void stop() {
        intake.setIntakePower(0);
    }
}
