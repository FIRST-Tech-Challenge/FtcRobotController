package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Controller.MechanicalDriveBase;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Basic drive", group = "drive")
public class BasicDrive extends OpMode
{
    MechanicalDriveBase mecanumDriveBase;

    @Override
    public void init()
    {
        mecanumDriveBase = new MechanicalDriveBase(hardwareMap);
        telemetry.addData("Initialized", " Press start");
        telemetry.update();
    }

    @Override
    public void loop()
    {
        mecanumDriveBase.gamepadController(gamepad1);
        mecanumDriveBase.driveBaseTelemetry(telemetry);
    }
}
