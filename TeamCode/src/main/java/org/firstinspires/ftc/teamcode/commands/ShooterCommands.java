package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.core.Command;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftTele;
import org.firstinspires.ftc.teamcode.toolkit.ftcdashboard.DashboardConstants;
import org.firstinspires.ftc.teamcode.toolkit.misc.Utils;

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
            shooter.setShooterVelocity(robot.highGoalVelocity);
        }

        if(opMode.gamepad2.b) {
            shooter.setShooterPower(0);
        }

        if(opMode.gamepad2.y || robot.shootingState == UpliftRobot.ShootingState.PREPARING_HIGHGOAL) {
            shooter.setShooterVelocity(robot.highGoalVelocity);
        }

        if(opMode.gamepad2.x || opMode.gamepad2.right_bumper || robot.shootingState == UpliftRobot.ShootingState.PREPARING_POWERSHOT) {
            shooter.setShooterVelocity(robot.powerShotVelocity);
        }

        if(robot.shootingState == UpliftRobot.ShootingState.DONE_SHOOTING) {
            shooter.setShooterPower(0.1);
        }

//        if (robot.shooterSensor.getDistance(DistanceUnit.CM) < 5){
//            while(robot.shooterSensor.getDistance(DistanceUnit.CM) < 5){
//                Utils.sleep(1);
//            }
//            robot.shotCount += 1;
//        }

//        robot.highGoalVelocity = DashboardConstants.targetVel;

    }

    @Override
    public void stop() {
        shooter.setShooterPower(0);
    }
}
