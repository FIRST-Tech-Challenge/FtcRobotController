package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.Commands.ManualDriveCommand;
import org.firstinspires.ftc.teamcode.Common.MyRobot;

@TeleOp(name = "Tank Drive TeleOp")
public class TankDriveTeleOp extends OpMode {

    private MyRobot robot;

    @Override
    public void init() {
        robot = new MyRobot(this);
        CommandScheduler.getInstance().setDefaultCommand(
                robot.driveSubsystem,
                new ManualDriveCommand(robot)
        );
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void stop() {
        robot.driveSubsystem.stop();
        CommandScheduler.getInstance().reset();
    }
}
