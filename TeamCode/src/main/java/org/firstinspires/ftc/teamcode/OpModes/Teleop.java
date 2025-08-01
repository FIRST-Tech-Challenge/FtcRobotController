package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Util.Subsystems.DoubleMotorLift;
import org.firstinspires.ftc.teamcode.Util.Subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.Util.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.Util.Subsystems.OuttakeClaw;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Lambda;


//Written by Noah Nottingham - 6566 Circuit Breakers
@Mercurial.Attach
@DoubleMotorLift.Attach
@IntakeClaw.Attach
@OuttakeClaw.Attach
@MecDrive.Attach
@Config //Allows ALL PUBLIC STATIC VARIABLES to be monitored on FTC Dash.
@TeleOp(name = "TeleOp", group = "Driver") //The name and group
//@Disabled //How you would disable/enable an opmode from appearing on the DS
public class Teleop extends OpMode {


    Follower follower;
    private int liftTarget = 0;


    @Override
    public void init() {




        Mercurial.gamepad1().a()
                .onTrue(new Lambda("Intake Close Claw"));
        Mercurial.gamepad1().b()
                .onTrue(new Lambda("Intake Open Claw"));
        Mercurial.gamepad1().x()
                .onTrue(new Lambda("Intake Toggle Claw"));
        Mercurial.gamepad1().rightBumper()
                .onTrue(MecDrive.slow())
                .onFalse(MecDrive.fast()); //No clue if this works or not




    }




    @Override
    public void init_loop() {

    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {


        liftTarget += (int) gamepad2.left_stick_y * 10;
        liftTarget = Math.max(Math.min(liftTarget,1500), 0);
        DoubleMotorLift.setLiftTarget(liftTarget);





    }
}
