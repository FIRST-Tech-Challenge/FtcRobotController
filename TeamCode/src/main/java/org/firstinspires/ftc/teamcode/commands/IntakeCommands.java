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

    public IntakeCommands(UpliftTele opMode, UpliftRobot robot, IntakeSubsystem intakeSubsystem) {
        super(opMode, intakeSubsystem);
        this.intake = intakeSubsystem;
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
        intake.setIntakePower(Range.clip(opMode.gamepad2.left_stick_y, -1, 1) * 0.8);
    }

    @Override
    public void stop() {
        intake.setIntakePower(0);
    }
}
