package org.firstinspires.ftc.teamcode.CenterStageRobot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.OuttakeSusystem;

public class OuttakeCommand extends CommandBase {
    private OuttakeSusystem outtakeSusystem;

    public enum Action {
        OPEN,
        CLOSE,
        EXTREME,
        TOOGLE
    }

    private Action action;

    public SequentialCommandGroup openOuttake() {
        return new SequentialCommandGroup(
                new InstantCommand(outtakeSusystem::go_outtake_first, outtakeSusystem),
                new WaitCommand(80),
                new InstantCommand(outtakeSusystem::go_outtake_second, outtakeSusystem)
        );
    }

    public SequentialCommandGroup closeOuttake() {
        return new SequentialCommandGroup(
                new InstantCommand(outtakeSusystem::go_intake_second, outtakeSusystem),
                new WaitCommand(80),
                new InstantCommand(outtakeSusystem::go_intake_first, outtakeSusystem)
        );
    }

    public SequentialCommandGroup openExtremeOuttake() {
        return new SequentialCommandGroup(
                new InstantCommand(outtakeSusystem::go_extreme_first, outtakeSusystem),
                new WaitCommand(70),
                new InstantCommand(outtakeSusystem::go_extreme_second, outtakeSusystem)
        );
    }

    public OuttakeCommand(OuttakeSusystem outtakeSusystem, Action action) {
        this.outtakeSusystem = outtakeSusystem;
        this.action = action;

        addRequirements(this.outtakeSusystem);
    }

    @Override
    public void initialize() {
        if (action == Action.OPEN) {
            openOuttake().schedule();
        } else if (action == Action.CLOSE) {
            closeOuttake().schedule();
        } else if (action == Action.EXTREME) {
            openExtremeOuttake().schedule();
        } else if (action == Action.TOOGLE) {
            if(outtakeSusystem.getState() == OuttakeSusystem.State.INTAKE) {
                openOuttake().schedule();
            } else {
                closeOuttake().schedule();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
