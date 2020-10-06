package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="Control Hub Test", group="Testiest")

//This will serve as a test of Control Hub program loading and running

public class ControlHubTest extends OpMode {
    @Override
    public void init() {
        telemetry.addData("Hello!","This is a test.");
    }

    @Override
    public void loop() {
        telemetry.addData("Ok","Now, wait for the timer.");
    }
}
