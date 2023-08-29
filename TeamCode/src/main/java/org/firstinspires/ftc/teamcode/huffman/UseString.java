package org.firstinspires.ftc.teamcode.huffman;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class UseString extends OpMode {
    @Override
    public void init() {
        String myName = "Asher Huffman";
        int grade = 9;
        telemetry.addData("Hello", myName);
        telemetry.addData("Grade", grade);
    }
    @Override
    public void loop() {

    }
}