package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "CB Prototype", group = "Teleop")
public class PrototypeCode extends OpMode
{
    public DcMotor motorOne;

    @Override
    public void init()
    {
        motorOne = hardwareMap.get(DcMotor.class, "Motor One");
    }

    @Override
    public void loop()
    {
        if(gamepad1.a)
            motorOne.setPower(.5);
        else
            motorOne.setPower(0);
    }
}