package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util.IntegratedCommands;
import org.firstinspires.ftc.teamcode.Util.Subsystems.IntakeLinkage;
import org.firstinspires.ftc.teamcode.Util.Subsystems.OuttakePivot;
import org.firstinspires.ftc.teamcode.Util.UniConstants;
import org.firstinspires.ftc.teamcode.Util.Subsystems.DoubleMotorLift;
import org.firstinspires.ftc.teamcode.Util.Subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.Util.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.Util.Subsystems.OuttakeClaw;

import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;


//Written by Noah Nottingham - 6566 Circuit Breakers
@Mercurial.Attach
@DoubleMotorLift.Attach
@IntakeClaw.Attach
@OuttakeClaw.Attach
@OuttakePivot.Attach
@MecDrive.Attach
@IntakeLinkage.Attach
@TeleOp(name = "TeleOp", group = "Driver") //The name and group
//@Disabled //How you would disable/enable an opmode from appearing on the DS
public class Teleop extends OpMode {


    private int liftTarget = 0;
    BoundGamepad gp1;
    BoundGamepad gp2;

    IntegratedCommands commands;

    UniConstants.loggingState logState = UniConstants.loggingState.ENABLED;

    @Override
    public void init() {



        gp1 = Mercurial.gamepad1();
        gp2 = Mercurial.gamepad2();

        gp1.a().onTrue(OuttakeClaw.toggleClaw());
        gp1.b().onTrue(OuttakeClaw.toggleRotation());
        gp1.x().onTrue(commands.toggleIntakeLinkage);
        gp1.rightBumper()
                .onTrue(MecDrive.slow())
                .onFalse(MecDrive.fast());
        gp1.dpadDown().onTrue(commands.outtakePivotDown);
        gp1.dpadUp().onTrue(OuttakePivot.pivotUp());
        gp1.dpadLeft().onTrue(OuttakePivot.pivotBar());
        gp1.dpadRight().onTrue(OuttakePivot.pivotBasket());





        gp2.dpadDown().onTrue(commands.outtakeSlidesHome);
        gp2.dpadUp().onTrue(DoubleMotorLift.goToLiftTarget(DoubleMotorLift.HeightStates.MIDDLE));
        gp2.dpadLeft().onTrue(DoubleMotorLift.goToLiftTarget(DoubleMotorLift.HeightStates.BASKET));
        gp2.dpadRight().onTrue(DoubleMotorLift.goToLiftTarget(DoubleMotorLift.HeightStates.BAR));







        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


    }

    @Override
    public void loop() {



        OuttakeClaw.log(logState);
        IntakeClaw.log(logState);
        MecDrive.log(logState);
        DoubleMotorLift.log(logState);
        IntakeLinkage.log(UniConstants.loggingState.EXTREME);

        telemetry.update();





    }
}
