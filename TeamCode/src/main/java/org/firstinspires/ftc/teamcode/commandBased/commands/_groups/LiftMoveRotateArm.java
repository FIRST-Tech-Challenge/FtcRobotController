package org.firstinspires.ftc.teamcode.commandBased.commands._groups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.classes.triggers.TriggerCommandBase;
import org.firstinspires.ftc.teamcode.classes.triggers.TriggerCommandGroup;
import org.firstinspires.ftc.teamcode.classes.triggers.TriggerCommandGroupBase;
import org.firstinspires.ftc.teamcode.commandBased.commands.arm.MoveArmToAngle;
import org.firstinspires.ftc.teamcode.commandBased.commands.elevator.MoveElevatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.MoveRotatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.RotatorSubsystem;

public class LiftMoveRotateArm extends TriggerCommandGroup {

    public MoveElevatorToPosition moveEle;
    public MoveArmToAngle moveArm;
    public MoveRotatorToPosition moveRot;

    public LiftMoveRotateArm(
            ElevatorSubsystem ele,
            ArmSubsystem arm,
            RotatorSubsystem rot,
            double armAngle,
            double armVelo,
            double armAccel,
            double eleTarget,
            double rotAngle
    ) {

//        moveEle = new MoveElevatorToPosition(ele, eleTarget);
//        moveArm = new MoveArmToAngle(arm, armAngle, armVelo, armAccel);
//        moveRot = new MoveRotatorToPosition(rot, rotAngle);
        addCommands(
                new MoveElevatorToPosition(ele, eleTarget),
                new MoveArmToAngle(arm, armAngle, armVelo, armAccel),
                new MoveRotatorToPosition(rot, rotAngle)
        );
        addRequirements(ele, arm, rot);
    }

    public int getCmdIndex() {
        return m_currentCommandIndex;
    }
}
