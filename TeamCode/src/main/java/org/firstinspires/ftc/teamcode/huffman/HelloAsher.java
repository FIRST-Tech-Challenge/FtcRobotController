package org.firstinspires.ftc.teamcode.huffman;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous()
@Disabled
public class HelloAsher extends OpMode {
    @Override
    public void init() {

        telemetry.addData("Hello","Asher");
    }

    @Override
    public void loop() {

    }
}
