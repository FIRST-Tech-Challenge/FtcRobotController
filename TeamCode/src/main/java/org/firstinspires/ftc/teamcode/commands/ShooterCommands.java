package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.core.Command;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftTele;

public class ShooterCommands extends Command {

    public ShooterSubsystem shooter;
    public UpliftTele opMode;
    Thread velThread;
    UpliftRobot robot;

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

        if(opMode.gamepad2.a){
            shooter.setShooterPower(1);
        }

        if(opMode.gamepad2.b) {
            shooter.setShooterPower(0);
        }

        if(opMode.gamepad2.y) {
            shooter.setShooterVelocity(robot.highGoalVelocity);
        }

        if(opMode.gamepad2.x || opMode.gamepad2.right_bumper || robot.shootingState == UpliftRobot.ShootingState.PREPARING_POWERSHOT) {
            shooter.setShooterVelocity(robot.powerShotVelocity);
        }

        if(robot.shootingState == UpliftRobot.ShootingState.DONE_SHOOTING) {
            shooter.setShooterPower(0.1);
        }

        if(opMode.gamepad2.left_bumper) {
            shooter.setShooterVelocity(robot.highGoalVelocity);
        }

    }

    @Override
    public void stop() {
        shooter.setShooterPower(0);
    }
}
