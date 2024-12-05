package org.firstinspires.ftc.teamcode.others;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class telemetryCounterOpmode extends OpMode {
    int count = 0;

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        if (count < 1000) {
            telemetry.addData("count", "the new value is " + count);
            count ++;
        }
    }
}
