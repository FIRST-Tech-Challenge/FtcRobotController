package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class HelloWorldTeam extends OpMode
{
    @Override
    public void init() {
        telemetry.addData("Hello", "World Team");
    }

    @Override
    public void loop() {

    }
}