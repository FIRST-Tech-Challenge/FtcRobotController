package org.firstinspires.ftc.teamcode.CART_team;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class HelloWorld extends OpMode
{
    @Override
    public void init() {
        telemetry.addData("Hello", "CART Team");
    }

    @Override
    public void loop() {

    }
}