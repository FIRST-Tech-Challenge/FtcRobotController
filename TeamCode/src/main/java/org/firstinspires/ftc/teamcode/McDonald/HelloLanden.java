package org.firstinspires.ftc.teamcode.McDonald;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous
public class HelloLanden extends OpMode {
    @Override
    public void init() {
        telemetry.addData("Hello","Landen");
    }

    @Override
    public void loop() {

    }
}
