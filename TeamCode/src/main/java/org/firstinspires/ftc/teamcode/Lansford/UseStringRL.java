package org.firstinspires.ftc.teamcode.Lansford;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
@Disabled
public class UseStringRL extends OpMode {
    @Override
    public void init() {
        String myName = "Rydon Lansford";// this is the name of the string

        telemetry.addData("Hello", myName);
    }

    @Override
    public void loop() {

    }
}
