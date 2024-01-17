package org.firstinspires.ftc.teamcode.CenterStageRobot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.OuttakeSusystem;

public class OuttakeCommand extends SequentialCommandGroup {
    OuttakeSusystem outtakeSusystem;
    public OuttakeCommand(OuttakeSusystem outtakeSusystem) {
        this.outtakeSusystem = outtakeSusystem;
        addRequirements(this.outtakeSusystem);
    }
    @Override
    public void initialize() {
        outtakeSusystem.go_outtake_second();
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
