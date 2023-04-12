package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team6220_PowerPlay.BaseTeleOp;

@Disabled
@TeleOp(name = "SlideMeasuringTest")
public class SlideMeasuringTest extends BaseTeleOp {
    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        while (opModeIsActive()) {
            driveSlidesWithController();
            driveGrabberWithController();
            // :)
            driveLEDs();

            telemetry.addData("slide position (ticks)", motorLeftSlides.getCurrentPosition());
            telemetry.update();
        }
    }
}
