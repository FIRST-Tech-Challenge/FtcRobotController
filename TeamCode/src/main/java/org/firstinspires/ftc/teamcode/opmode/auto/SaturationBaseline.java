package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.vision.pipeline.HSVSaturationPipeline;
import org.firstinspires.ftc.teamcode.vision.util.FieldPosition;


@Autonomous(name="Saturation Baseline", group="OpMode")
public class SaturationBaseline extends AutoBase{

    @Override
    public void init() {



        setFieldPosition(FieldPosition.BLUE_FIELD_LEFT);

        // finally do the init in the AutoBase that will set up the camera and motors
        super.init();
    }


    @Override
    public void init_loop() {
        telemetry.addData("LSpikeSaturation",getLeftSpikeSaturation());
        telemetry.addData("RSpikeSaturation",getRightSpikeSaturation());
        telemetry.addData("CSpikeSaturation",getCenterSpikeSaturation());

        telemetry.update();
    }

    @Override
    public void loop(){

    }
}
