package org.firstinspires.ftc.teamcode.WillKirby;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class UseString extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        String myName = "RoboKai";
        telemetry.addData("Hello", myName);
        telemetry.update();
    }
}



