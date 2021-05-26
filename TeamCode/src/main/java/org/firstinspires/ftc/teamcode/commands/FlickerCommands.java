package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.subsystems.FlickerSubsystem;
import org.firstinspires.ftc.teamcode.toolkit.core.Command;
import org.firstinspires.ftc.teamcode.toolkit.core.UpliftTele;

public class FlickerCommands extends Command {

    public FlickerSubsystem flicker;
    public UpliftTele opMode;
    UpliftRobot robot;

    public FlickerCommands(UpliftTele opMode, UpliftRobot robot, FlickerSubsystem flickerSubsystem) {
        super(opMode, flickerSubsystem);
        this.flicker = flickerSubsystem;
        this.opMode = opMode;
        this.robot = robot;
    }

    @Override
    public void init() {
        // set flicker to start position (back)
        flicker.setFlickerOut();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        if(opMode.gamepad1.right_bumper) {
            flicker.flickRing();
        }

        // Method for shooting (only flicks 3 rings, then tells transfer to drop and shooter to slow)
        if(opMode.gamepad1.y) {
            for(int i = 0; i < 3; i++) {
                double initialTime = System.currentTimeMillis();
                while(!robot.velocityData.isHighGoalShooterReady() && (System.currentTimeMillis() - initialTime) < 2000 && !robot.operatorCancel && opMode.opModeIsActive()) {
                    robot.safeSleep(1);
                    if(robot.driverCancel || robot.operatorCancel) {
                        flicker.setFlickerOut();
                        robot.setStickState(UpliftRobot.StickState.DOWN);
                        Log.i("flicker", "CANCELLED");
                        return;
                    }
                }
                flicker.flickRing();
            }
            if(!robot.driverCancel) {
                robot.setShootingState(UpliftRobot.ShootingState.DONE_SHOOTING);
            }
        }

        if(robot.shootingState == UpliftRobot.ShootingState.SHOOTING_PS1) {
            while(!robot.velocityData.isPowerShotShooterReady() && opMode.opModeIsActive()) {
                if(!robot.safeSleep(1)) return;
            }
            flicker.flickRing();
            robot.setShootingState(UpliftRobot.ShootingState.DONE_PS1);
        }

        if(robot.shootingState == UpliftRobot.ShootingState.SHOOTING_PS2) {
            while(!robot.velocityData.isPowerShotShooterReady() && opMode.opModeIsActive()) {
                if(!robot.safeSleep(1)) return;
            }
            flicker.flickRing();
            robot.setShootingState(UpliftRobot.ShootingState.DONE_PS2);
        }

        if(robot.shootingState == UpliftRobot.ShootingState.SHOOTING_PS3) {
            while(!robot.velocityData.isPowerShotShooterReady() && opMode.opModeIsActive()) {
                if(!robot.safeSleep(1)) return;
            }
            flicker.flickRing();
            robot.setShootingState(UpliftRobot.ShootingState.DONE_PS3);
        }

    }

    @Override
    public void stop() {

    }
}
