package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util.UniConstants;
import org.firstinspires.ftc.teamcode.Util.Subsystems.DoubleMotorLift;
import org.firstinspires.ftc.teamcode.Util.Subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.Util.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.Util.Subsystems.OuttakeClaw;

import dev.frozenmilk.mercurial.Mercurial;

@OuttakeClaw.Attach
@IntakeClaw.Attach
@MecDrive.Attach
@DoubleMotorLift.Attach
@Mercurial.Attach
@TeleOp(name = "Debugging Teleop", group = "DEBUG")
public class DebuggingTeleop extends OpMode {

    UniConstants.loggingState logState = UniConstants.loggingState.EXTREME;


    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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
