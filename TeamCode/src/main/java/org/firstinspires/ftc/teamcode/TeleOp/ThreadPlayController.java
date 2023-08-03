package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controller.MechanicalDriveBase;

@TeleOp(name = "threadcode", group = "threadplay")
public class ThreadPlayController extends OpMode
{
    MechanicalDriveBase mecanumDriveBase;
    ColorSensorClass colorSensorClass;

    @Override
    public void init()
    {
        colorSensorClass = new ColorSensorClass(hardwareMap);
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
