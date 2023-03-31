package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team6220_PowerPlay.BaseTeleOp;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

@TeleOp(name = "SlideHeightTest")
public class SlideHeightTest extends BaseTeleOp {

    int[] heights = new int[] {0, 117, 238, 383, 537, Constants.SLIDE_LOW, Constants.SLIDE_MEDIUM, Constants.SLIDE_HIGH, Constants.SLIDE_TOP};
    int selectedHeight = 0;
    boolean newButtonPressed = true;

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        while (opModeIsActive()) {
            //driveSlidesWithController();
            driveGrabberWithController();
            // :)
            driveLEDs();

            if(gamepad2.dpad_up & newButtonPressed & selectedHeight < heights.length-1) {
                newButtonPressed = false;
                selectedHeight++;
            }
            else if(gamepad2.dpad_down & newButtonPressed & selectedHeight > 0) {
                newButtonPressed = false;
                selectedHeight--;
            }

            if(!(gamepad2.dpad_up | gamepad2.dpad_down)) {
                newButtonPressed = true;
            }

            driveSlides(heights[selectedHeight]);

            telemetry.addData("slide position (ticks)", motorLeftSlides.getCurrentPosition());
            telemetry.addData("slide stack position index", selectedHeight);
            telemetry.addData("slide stack position ticks", heights[selectedHeight]);
            telemetry.addData("new button press", newButtonPressed);
            telemetry.update();
        }
    }
}
