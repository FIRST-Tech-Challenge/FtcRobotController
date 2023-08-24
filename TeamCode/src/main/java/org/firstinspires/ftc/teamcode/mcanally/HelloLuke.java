package org.firstinspires.ftc.teamcode.mcanally;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()

public class HelloLuke extends OpMode {

    @Override
    public void init() {telemetry.addData("Hello","Luke"); }

    public void loop(){
    }
}
