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

    IntegratedCommands commands = new IntegratedCommands();

    UniConstants.loggingState logState = UniConstants.loggingState.ENABLED;

    @Override
    public void init() {



        gp1 = Mercurial.gamepad1();
        gp2 = Mercurial.gamepad2();

        gp1.a().onTrue(IntakeClaw.toggleClaw());
        gp1.b().onTrue(IntakeClaw.toggleHorizontal());
        gp1.x().onTrue(IntakeClaw.toggleVertical());
        gp1.y().onTrue(commands.transfer);

        gp1.rightBumper()
                .onTrue(MecDrive.slow())
                .onFalse(MecDrive.fast());
        gp1.leftBumper().onTrue(commands.toggleIntakeLinkage);

        gp1.dpadDown().onTrue(commands.outtakePivotDown);
        gp1.dpadUp().onTrue(OuttakePivot.pivotUp());
        gp1.dpadLeft().onTrue();
        gp1.dpadRight().onTrue();
        gp1.share().onTrue(commands.init);


        gp2.a().onTrue(OuttakeClaw.toggleClaw());
        gp2.b().onTrue(OuttakeClaw.toggleRotation());

        gp2.dpadDown().onTrue(commands.outtakeSlidesHome);
        gp2.dpadUp().onTrue();
        gp2.dpadLeft().onTrue(commands.outtakeBasket);
        gp2.dpadRight().onTrue(commands.outtakeBar);







        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


    }

    @Override
    public void start(){

        commands.init.schedule();

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
