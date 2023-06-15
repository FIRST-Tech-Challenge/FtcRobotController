package org.firstinspires.ftc.teamcode.commandBased.commands._groups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.classes.command.triggers.TriggeredCommandGroup;
import org.firstinspires.ftc.teamcode.commandBased.commands.intake.SetIntakePower;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.IntakeSubsystem;

public class MoveRotateArm extends SequentialCommandGroup {

    public MoveRotateArm(
            ArmSubsystem arm,
            IntakeSubsystem intake,
            double armAngle,
            double velo,
            double accel,
            double intakePower
    ) {
        addCommands(
                new SetIntakePower(intake, intakePower)
        );


//        new SequentialCommandGroup(
//                new drive(),
//                new TriggeredCommandGroup(
//                        new elevator(),
//                        new arm(),
//                        new rotator(),
//                ),
//                new outtake(),
//                new drive()
//        )



    }
}
