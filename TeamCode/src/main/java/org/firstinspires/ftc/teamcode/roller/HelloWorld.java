package org.firstinspires.ftc.teamcode.roller;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Autonomous
public class HelloWorld extends OpMode {
    @Override
    public void init(){
        telemetry.addData("Hello","Mr. Roller");
    }

    @Override
    public void loop(){

    }
}
