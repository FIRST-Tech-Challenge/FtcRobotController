package org.firstinspires.ftc.teamcode.huffman;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp() //Assigns the OpMode to the TeleOp dropdown on Driver Station
public class UseStringAH extends OpMode {
    @Override
    public void init() {
        //creates a String class to store myName
        String myName = "Asher Huffman";

        //creates an integer variable to store grade
        int grade = 9;

        //writes data stored in String to Driver Station, adds "Hello" caption
        telemetry.addData("Hello", myName);

        //writes data stored in int to Driver Station, adds "Grade" caption
        telemetry.addData("Grade", grade);
    }
    @Override
    public void loop() {
//required even if empty
    }
}