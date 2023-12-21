package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.vision.pipeline.HSVSaturationPipeline;
import org.firstinspires.ftc.teamcode.vision.util.FieldPosition;


@Autonomous(name="Saturation Baseline", group="OpMode")
public class Baseline extends AutoBase{

    @Override
    public void init() {

        // create the opencv pipeline we are going to use for this opmode
        hsvPipe = new HSVSaturationPipeline();

        hsvPipe.setFieldPosition(FieldPosition.BLUE_FIELD_LEFT);
        // give the AutoBase our pipeline so it can attach it to the camera
        setPipeline(hsvPipe);

        // finally do the init in the AutoBase that will set up the camera and motors
        super.init();
    }


    @Override
    public void init_loop() {
        telemetry.addData("LSpikeSaturation",hsvPipe.getLeftSpikeSaturation());
        telemetry.addData("RSpikeSaturation",hsvPipe.getRightSpikeSaturation());
        telemetry.addData("CSpikeSaturation",hsvPipe.getCenterSpikeSaturation());

        telemetry.update();
    }

    @Override
    public void loop(){

    }
}
