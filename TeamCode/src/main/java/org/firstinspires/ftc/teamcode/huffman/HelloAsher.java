package org.firstinspires.ftc.teamcode.huffman;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class HelloAsher extends OpMode {
    @Override
    public void init() {
        telemetry.addData("Hello","Asher");
    }
}
