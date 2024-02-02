package org.firstinspires.ftc.teamcode.CenterStageRobot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeArmSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.OuttakeSusystem;

public class IntakeCommand extends CommandBase {
    IntakeSubsystem intakeSubsystem;
    IntakeArmSubsystem intakeArmSubsystem;
    OuttakeSusystem outtakeSusystem;
    int state;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, IntakeArmSubsystem intakeArmSubsystem,
                         OuttakeSusystem outtakeSusystem, int state) {
        this.intakeSubsystem = intakeSubsystem;
        this.intakeArmSubsystem = intakeArmSubsystem;
        this.outtakeSusystem = outtakeSusystem;
        addRequirements(this.intakeSubsystem, this.intakeArmSubsystem, this.outtakeSusystem);

        this.state = state;
    }

    @Override
    public void initialize() {
        super.initialize();
    }
}
