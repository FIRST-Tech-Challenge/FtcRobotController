package org.firstinspires.ftc.teamcode.CenterStageRobot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.OuttakeSusystem;

public class OuttakeCommand extends CommandBase {
    OuttakeSusystem outtakeSusystem;
    public OuttakeCommand(OuttakeSusystem outtakeSusystem) {
        this.outtakeSusystem = outtakeSusystem;
        addRequirements(this.outtakeSusystem);
    }
    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
