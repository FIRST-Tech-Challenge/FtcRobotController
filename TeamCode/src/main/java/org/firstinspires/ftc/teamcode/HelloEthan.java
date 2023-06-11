package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class HelloEthan extends OpMode{
    @Override
    public void init() {
        telemetry.addData("Hello", "World");
    }
    @Override
    public  void loop() {

    }
}

