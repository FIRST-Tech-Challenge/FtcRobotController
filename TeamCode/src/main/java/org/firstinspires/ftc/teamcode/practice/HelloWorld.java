package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class HelloWorld extends OpMode {
    @Override
    public void init() {
        telemetry.addData("Hello", "James Rodvold"); // My First Comment
    }

    @Override
    public void loop() {

    }
    // Single Line Comment
    /*
    1. Hello: World, change the telemetry data to desplay "Hello: Your name"
    2. run this code Autonomous of your DS instead of TeleOpe
    @Disabled turns the class of code off
    some codes are alowed to be empty like void loop, and it is fine for the computer
    @TleOp turns code back on
     */
}

