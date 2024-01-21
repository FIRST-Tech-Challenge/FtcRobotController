package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.vision.util.FieldPosition;

/**
 * This is a special OpMode to gather the average saturation of the virtual spike line boxes.
 * We have put boxes around the spike lines to exclude noise that can interfere with getting a
 * proper reading.  We were having problems capturing the end of the field in our image processing
 * and it led to bad results.
 *
 * It is worth checking the camera stream when doing the baseline testing to make sure that the
 * boxes are enclosing the spike lines.
 *
 * Important note:  Currently, we are calibrating ONLY ON THE BLUE_FIELD_LEFT position.  If we
 * find that we need to calibrate for each field position, we will have to update this OpMode
 * This would also allow us to verify the spike line boxes
 */
@Autonomous(name="Saturation Baseline", group="OpMode")
public class SaturationBaseline extends AutoBase {

    @Override
    public void runOpMode() {
        // finally do the init in the AutoBase that will set up the camera and motors
        super.runOpMode();

        // Change this if you want to calibrate for a different field position
        setFieldPosition(FieldPosition.BLUE_FIELD_LEFT);



        while (opModeInInit()) {
            telemetry.addData("LSpikeSaturation", getLeftSpikeSaturation());
            telemetry.addData("RSpikeSaturation", getRightSpikeSaturation());
            telemetry.addData("CSpikeSaturation", getCenterSpikeSaturation());


            telemetry.update();
        }
        while (opModeIsActive()) {

            // TODO: Here is an idea,  we could map buttons to the different field positions and print the results like in the init_loop
        }
    }
}
