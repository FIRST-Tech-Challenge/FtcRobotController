package org.firstinspires.ftc.teamcode.Util;

import org.firstinspires.ftc.teamcode.Util.Subsystems.DoubleMotorLift;
import org.firstinspires.ftc.teamcode.Util.Subsystems.IntakeLinkage;
import org.firstinspires.ftc.teamcode.Util.Subsystems.OuttakePivot;

import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.groups.Parallel;

public class IntegratedCommands {

    public Command toggleIntakeLinkage = new Parallel(
            OuttakePivot.pivotUp(),
            DoubleMotorLift.goToLiftTarget(DoubleMotorLift.HeightStates.MIDDLE),
            IntakeLinkage.toggleLinkage()
    );

    public Command outtakePivotDown = new Parallel(
            OuttakePivot.pivotDown(),
            DoubleMotorLift.goToLiftTarget(DoubleMotorLift.HeightStates.MIDDLE)
    );

    public Command outtakeSlidesHome = new Parallel(
            OuttakePivot.pivotUp(),
            DoubleMotorLift.home()
    );





}
