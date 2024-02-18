package org.firstinspires.ftc.teamcode.CenterStageRobot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.OuttakeSusystem;

public class OuttakeCommand extends SequentialCommandGroup {
    OuttakeSusystem outtakeSusystem;

    public enum OuttakeAction {
        OPEN,
        CLOSE,
        TOGGLE
    }

    private OuttakeAction action;

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

    public OuttakeCommand(OuttakeSusystem outtakeSusystem, OuttakeAction action) {
        this.outtakeSusystem = outtakeSusystem;
        this.action = action;

        addRequirements(this.outtakeSusystem);
    }
    @Override
    public void initialize() {
        if (action == OuttakeAction.OPEN) {
            openOuttake().schedule();
        } else if (action == OuttakeAction.CLOSE) {
            closeOuttake().schedule();
        } else if (action == OuttakeAction.TOGGLE) {
            if (outtakeSusystem.getState() == OuttakeSusystem.State.INTAKE) {
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
