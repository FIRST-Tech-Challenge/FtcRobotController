package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Controller.MecanumDriveBase;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "basic drive", group = "drive")
public class BasicDrive extends OpMode
{
    MecanumDriveBase mecanumDriveBase;
    @Override
    public void init()
    {
        mecanumDriveBase = new MecanumDriveBase(hardwareMap);
    }

    @Override
    public void loop()
    {
        mecanumDriveBase.gamepadController(gamepad1);
        mecanumDriveBase.driveBaseTelemetry(telemetry);
    }
}
