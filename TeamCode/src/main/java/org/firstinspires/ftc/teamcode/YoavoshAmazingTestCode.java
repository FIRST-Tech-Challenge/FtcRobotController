package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class YoavoshAmazingTestCode extends OpMode {
    // ints
    int counter = 0;
    @Override
    public void init() {

    }

    @Override
    public void loop() {

        telemetry.addData("counter", ++counter);
        telemetry.update();
    }
}
