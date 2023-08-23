package org.firstinspires.ftc.robotcontroller.McAnally;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class HelloLuke extends OpMode {
    @Override
    public void init() {
        telemetry.addData("Hello","World");
    }
    @Override
    public void loop() {
    }
}
