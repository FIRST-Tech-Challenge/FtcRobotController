package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CenterStageRobot.commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeArmSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.OuttakeSusystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.PixelFingerSubsystem;

public class AutonomousCommands {
    /*-------------------------------------------------------
    -FTCLib Commands-
    -------------------------------------------------------*/
    private OuttakeSusystem outtakeSusystem;
    private ElevatorSubsystem elevatorSubsystem;
    private ElevatorCommand elevatorCommand;
    private IntakeArmSubsystem intakeArmSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private PixelFingerSubsystem pixelFingerSubsystem;

    public AutonomousCommands(HardwareMap hm) {
        outtakeSusystem = new OuttakeSusystem(hm);
        elevatorSubsystem = new ElevatorSubsystem(hm, telemetry, () -> 0);
        intakeSubsystem = new IntakeSubsystem(hm, telemetry);
        intakeArmSubsystem = new IntakeArmSubsystem(hm);
        pixelFingerSubsystem = new PixelFingerSubsystem(hm);
    }

    public InstantCommand spikePixelTake(){
        return new InstantCommand(intakeArmSubsystem::LOCK_PIXEL, intakeArmSubsystem);
    }

    public InstantCommand spikeScoring(){
        return new InstantCommand(() -> intakeArmSubsystem.auto_pixel(6), intakeArmSubsystem);
    }
    public SequentialCommandGroup randomizationPixelElevator(){
        return new SequentialCommandGroup(
                new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.AUTO),
                new InstantCommand(outtakeSusystem::go_outtake_first, outtakeSusystem),
                new WaitCommand(80),
                new InstantCommand(outtakeSusystem::go_outtake_second, outtakeSusystem)
        );
    }

    public SequentialCommandGroup elevator(){
        return new SequentialCommandGroup(
                new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.LOW),
                new InstantCommand(outtakeSusystem::go_outtake_first, outtakeSusystem),
                new WaitCommand(80),
                new InstantCommand(outtakeSusystem::go_outtake_second, outtakeSusystem)
        );
    }

    public SequentialCommandGroup scoring_randomization(){
        return new SequentialCommandGroup(
                new InstantCommand(outtakeSusystem::wheel_release, outtakeSusystem),
                new WaitCommand(1000),
                new InstantCommand(outtakeSusystem::wheel_stop, outtakeSusystem),
                new InstantCommand(outtakeSusystem::go_intake_second, outtakeSusystem),
                new WaitCommand(80),
                new InstantCommand(outtakeSusystem::go_intake_first, outtakeSusystem),
                new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.LOADING)

        );
    }

    public SequentialCommandGroup scoring(){
        return new SequentialCommandGroup(
            new InstantCommand(outtakeSusystem::wheel_release, outtakeSusystem),
            new WaitCommand(2000),
            new InstantCommand(outtakeSusystem::wheel_stop, outtakeSusystem),
            new InstantCommand(outtakeSusystem::go_intake_second, outtakeSusystem),
            new WaitCommand(80),
            new InstantCommand(outtakeSusystem::go_intake_first, outtakeSusystem),
            new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.LOADING)

        );
    }

    public SequentialCommandGroup stackStationIntake(int index) {
        return new SequentialCommandGroup(
                new InstantCommand(intakeSubsystem::run),
                new InstantCommand(outtakeSusystem::wheel_grab),
                new WaitCommand(150),
                new SequentialCommandGroup(
                        new InstantCommand(()-> intakeArmSubsystem.auto_pixel(index), intakeArmSubsystem),
                        new WaitCommand(300),
                        new InstantCommand(()-> intakeArmSubsystem.auto_pixel(index - 1), intakeArmSubsystem),
                        new WaitCommand(600)
                ),
                new ParallelCommandGroup(
                        new InstantCommand(intakeSubsystem::stop),
                        new InstantCommand(()-> intakeArmSubsystem.auto_pixel(6), intakeArmSubsystem)
                )
        );
    }
}
