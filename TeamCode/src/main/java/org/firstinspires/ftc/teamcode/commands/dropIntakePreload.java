package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.robot.Command;
import org.firstinspires.ftc.teamcode.subsystems.CrabRobot;

public class dropIntakePreload implements Command {

    CrabRobot robot;
    //double duration;
    NanoClock clock;

    public dropIntakePreload(CrabRobot robot) {
        this.robot= robot;
        clock = NanoClock.system();
    }

    @Override
    public void start() {
        this.robot.intake.intakeState = 4;
    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {
    }

    @Override
    public boolean isCompleted() {
        return (this.robot.intake.intakeState == 0);
    }
}
