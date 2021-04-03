package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.core.Command;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftTele;
import org.firstinspires.ftc.teamcode.toolkit.misc.Utils;

public class ShooterCommands extends Command {

    public ShooterSubsystem shooter;
    public UpliftTele opMode;
    Thread velThread;
    UpliftRobot robot;
    boolean shooterSwitchPressed = false;

    public ShooterCommands(UpliftTele opMode, UpliftRobot robot, ShooterSubsystem shooterSubsystem) {
        super(opMode, shooterSubsystem);
        this.shooter = shooterSubsystem;
        this.opMode = opMode;
        this.robot = robot;
    }

    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        // Toggle Button for shooter type (highgoal if true, powershot if false)
        if(opMode.gamepad2.x) {
            if(!shooterSwitchPressed) {
                robot.highGoalMode = !robot.highGoalMode;
                shooterSwitchPressed = true;
            }
        } else {
            shooterSwitchPressed = false;
        }

        if(opMode.gamepad2.a || opMode.gamepad2.y) {
            if(robot.highGoalMode) {
                shooter.setShooterVelocity(robot.highGoalVelocity);
            } else {
                shooter.setShooterVelocity(robot.powerShotVelocity);
            }
        }

        if(opMode.gamepad2.b) {
            shooter.setShooterPower(0);
        }

        if(robot.shootingState == UpliftRobot.ShootingState.DONE_SHOOTING) {
            shooter.setShooterPower(0);
        }

//        robot.highGoalVelocity = DashboardConstants.targetVel;

    }

    @Override
    public void stop() {
        shooter.setShooterPower(0);
    }
}
