package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
@MecDrive.Attach
@TeleOp(name = "TeleOp", group = "Driver") //The name and group
//@Disabled //How you would disable/enable an opmode from appearing on the DS
public class Teleop extends OpMode {


    private int liftTarget = 0;
    BoundGamepad gp1;
    BoundGamepad gp2;

    UniConstants.loggingState logState = UniConstants.loggingState.ENABLED;

    @Override
    public void init() {



        gp1 = Mercurial.gamepad1();
        gp2 = Mercurial.gamepad2();

        gp1.a()
                .onTrue(OuttakeClaw.closeClaw());
        gp1.b()
                .onTrue(OuttakeClaw.openClaw());
        gp1.x()
                .onTrue(OuttakeClaw.toggleClaw());
        gp1.rightBumper()
                .onTrue(MecDrive.slow())
                .onFalse(MecDrive.fast()); //No clue if this works or not

        gp2.dpadDown().onTrue(DoubleMotorLift.home());
        gp2.dpadUp().onTrue(DoubleMotorLift.goToLiftTarget(DoubleMotorLift.HeightStates.MIDDLE));
        gp2.dpadLeft().onTrue(DoubleMotorLift.goToLiftTarget(DoubleMotorLift.HeightStates.BASKET));
        gp2.dpadRight().onTrue(DoubleMotorLift.goToLiftTarget(DoubleMotorLift.HeightStates.BAR));


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


    }




    @Override
    public void init_loop() {

    }

    @Override
    public void start(){



    }

    @Override
    public void loop() {



        OuttakeClaw.log(logState);
        IntakeClaw.log(logState);
        MecDrive.log(logState);
        DoubleMotorLift.log(logState);
        telemetry.update();





    }
}
