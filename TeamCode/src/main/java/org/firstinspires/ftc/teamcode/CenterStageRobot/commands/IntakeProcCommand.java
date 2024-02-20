package org.firstinspires.ftc.teamcode.CenterStageRobot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeArmSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.OuttakeSusystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.PixelColorDetectorSubsystem;

public class IntakeProcCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    private IntakeArmSubsystem intakeArmSubsystem;
    private OuttakeSusystem outtakeSusystem;
    private ElevatorSubsystem elevatorSubsystem;
    private PixelColorDetectorSubsystem pixelColorDetectorSubsystem;

    public IntakeProcCommand(IntakeSubsystem intakeSubsystem, IntakeArmSubsystem intakeArmSubsystem,
                             OuttakeSusystem outtakeSusystem, ElevatorSubsystem elevatorSubsystem,
                             PixelColorDetectorSubsystem pixelColorDetectorSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.intakeArmSubsystem = intakeArmSubsystem;
        this.outtakeSusystem = outtakeSusystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.pixelColorDetectorSubsystem = pixelColorDetectorSubsystem;

        addRequirements(this.intakeSubsystem, this.intakeArmSubsystem, this.outtakeSusystem, this.elevatorSubsystem, this.pixelColorDetectorSubsystem);
    }

    @Override
    public void initialize() {
        if(intakeSubsystem.getState() == IntakeSubsystem.State.RESTING) {
            new ParallelCommandGroup(
                new InstantCommand(intakeArmSubsystem::lowerArm, intakeArmSubsystem),
                new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.LOADING)
//                new OuttakeCommand(outtakeSusystem, OuttakeCommand.OuttakeAction.CLOSE)
            ).schedule();

            new ParallelCommandGroup(
                    new InstantCommand(outtakeSusystem::wheel_grab)
//                    new IntakeCommand(intakeSubsystem, pixelColorDetectorSubsystem, )
            ).schedule();
        } else if(intakeSubsystem.getState() == IntakeSubsystem.State.LOADING) {
            new SequentialCommandGroup(
                    new InstantCommand(outtakeSusystem::wheel_stop),
                    new InstantCommand(intakeArmSubsystem::raiseArm),
                    new WaitCommand(200),
                    new InstantCommand(intakeSubsystem::reverse),
                    new WaitCommand(600),
                    new InstantCommand(intakeSubsystem::stop, intakeSubsystem)
            ).schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
