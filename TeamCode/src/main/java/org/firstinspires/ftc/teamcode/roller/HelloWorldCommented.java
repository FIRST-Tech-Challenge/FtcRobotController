package org.firstinspires.ftc.teamcode.roller;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous
public class HelloWorldCommented extends OpMode {
    /**
     * This is called when the driver presses INIT
     */
    @Override
    public void init(){
        // this sends data to the driver station
        telemetry.addData("Hello","Mr. Roller");
    }
    /**
     * This is called repeatedly while OpMode is playing
     */
    @Override
    public void loop(){
        // intentionally left blank
    }
}
