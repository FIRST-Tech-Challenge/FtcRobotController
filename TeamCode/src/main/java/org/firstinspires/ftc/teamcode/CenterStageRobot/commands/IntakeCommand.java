package org.firstinspires.ftc.teamcode.CenterStageRobot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeArmSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.OuttakeSusystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.PixelColorDetectorSubsystem;

public class IntakeCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    private IntakeArmSubsystem intakeArmSubsystem;
    private OuttakeSusystem outtakeSusystem;
    private PixelColorDetectorSubsystem pixelColorDetectorSubsystem;
    public IntakeCommand(IntakeSubsystem intakeSubsystem, PixelColorDetectorSubsystem pixelColorDetectorSubsystem,
                         OuttakeSusystem outtakeSusystem, IntakeArmSubsystem intakeArmSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.pixelColorDetectorSubsystem = pixelColorDetectorSubsystem;
        this.intakeArmSubsystem = intakeArmSubsystem;
        this.outtakeSusystem = outtakeSusystem;

        addRequirements(intakeSubsystem, pixelColorDetectorSubsystem, outtakeSusystem, intakeArmSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.run();
    }

    @Override
    public void end(boolean interrupted) {
        new SequentialCommandGroup(
                new InstantCommand(outtakeSusystem::wheel_stop),
                new InstantCommand(intakeArmSubsystem::raiseArm),
                new WaitCommand(200),
                new InstantCommand(intakeSubsystem::reverse, intakeSubsystem),
                new WaitCommand(600),
                new InstantCommand(intakeSubsystem::stop, intakeSubsystem)
        ).schedule();
//        intakeSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return pixelColorDetectorSubsystem.isFrontPixel(); // || intakeSubsystem.isStalled();
    }
}
