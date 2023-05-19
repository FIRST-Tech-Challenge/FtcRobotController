package org.firstinspires.ftc.teamcode.Outreach;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo  Locator", group = "programmer aid")
public class ServoLocator extends OpMode
{
    private Servo trigger;

    @Override
    public void init()
    {
        trigger = hardwareMap.get(Servo.class, "trigger");
    }

    @Override
    public void loop()
    {
        trigger.setPosition(1);
        telemetry.addData("position", trigger.getPosition() + " port" + trigger.getPortNumber());
        telemetry.update();
    }
}
