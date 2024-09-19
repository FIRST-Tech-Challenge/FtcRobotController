package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.controller.MechanicalDriveBase;

@TeleOp(name = "Basic Drive", group = "OpMOde")
public class BasicDrive extends OpMode
{
    MechanicalDriveBase mechanicalDriveBase;

    @Override
    public void init()
    {
        mechanicalDriveBase = new MechanicalDriveBase(hardwareMap);
    }

    @Override
    public void loop()
    {
        mechanicalDriveBase.gamepadController(gamepad1);
        mechanicalDriveBase.driveBaseTelemetry(telemetry);
    }
}
