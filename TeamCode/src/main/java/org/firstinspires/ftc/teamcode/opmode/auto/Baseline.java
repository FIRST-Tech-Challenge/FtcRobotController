package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="Saturation Baseline", group="OpMode")
public class Baseline extends AutoBase{

    @Override
    public void init() {
        super.init();

    }

    @Override
    public void init_loop() {
        telemetry.addData("LSpikeSaturation",pipeline.getLeftSpikeSaturation());
        telemetry.addData("RSpikeSaturation",pipeline.getRightSpikeSaturation());
        telemetry.addData("CSpikeSaturation",pipeline.getCenterSpikeSaturation());
        telemetry.update();
    }

    @Override
    public void loop(){

    }
}
