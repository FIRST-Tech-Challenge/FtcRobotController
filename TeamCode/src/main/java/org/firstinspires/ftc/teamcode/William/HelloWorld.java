package org.firstinspires.ftc.teamcode.William;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class HelloWorld extends OpMode
{
    @Override
    public void init() {
        telemetry.addData("Hello", "William");
    }

    @Override
    public void loop() {

    }
}
