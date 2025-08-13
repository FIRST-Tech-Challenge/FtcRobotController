package org.firstinspires.ftc.teamcode.Util;

import org.firstinspires.ftc.teamcode.Util.Subsystems.DoubleMotorLift;
import org.firstinspires.ftc.teamcode.Util.Subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.Util.Subsystems.IntakeLinkage;
import org.firstinspires.ftc.teamcode.Util.Subsystems.OuttakeClaw;
import org.firstinspires.ftc.teamcode.Util.Subsystems.OuttakePivot;

import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

public class IntegratedCommands {

    public Command toggleIntakeLinkage = new Sequential(
            new Parallel(
                OuttakePivot.pivotUp(),
                DoubleMotorLift.goToLiftTarget(DoubleMotorLift.HeightStates.MIDDLE)
            ),
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

    public Command outtakeBasket = new Sequential(
            DoubleMotorLift.goToLiftTarget(DoubleMotorLift.HeightStates.BASKET),
            OuttakePivot.pivotBasket()
    );

    public Command outtakeBar = new Parallel(
            DoubleMotorLift.goToLiftTarget(DoubleMotorLift.HeightStates.BAR),
            OuttakePivot.pivotBar()
    );

    public Command init = new Sequential(
            outtakeSlidesHome,
            DoubleMotorLift.goToLiftTarget(DoubleMotorLift.HeightStates.MIDDLE),
            IntakeLinkage.linkageIn(),
            new Parallel(
                    OuttakeClaw.closeClaw(),
                    OuttakeClaw.paraClaw(),
                    IntakeClaw.closeClaw(),
                    IntakeClaw.paraClaw(),
                    IntakeClaw.upClaw()
            )

    );

    public Command intakeReset = new Sequential(
            IntakeClaw.upClaw(),
            IntakeClaw.paraClaw(),
            IntakeLinkage.linkageIn(),
            new Wait(.25)
    );

    public Command outtakeReset = new Sequential(
            OuttakePivot.pivotUp(),
            OuttakeClaw.paraClaw(),
            outtakeSlidesHome
    );

    public Command reset = new Sequential(
            intakeReset,
            outtakeReset
    );

    public Command transfer = new Sequential(

            reset,
            new Wait(.125),
            new Parallel(
                IntakeLinkage.linkageTransfer(),
                DoubleMotorLift.goToLiftTarget(DoubleMotorLift.HeightStates.TRANSFER),
                    OuttakePivot.pivotTransfer(),
                    OuttakeClaw.openClaw()
            ),
            new Wait(.125),
            OuttakeClaw.closeClaw(),
            new Wait(.125),
            IntakeClaw.openClaw(),
            new Wait(.125),
            init

    );





}
