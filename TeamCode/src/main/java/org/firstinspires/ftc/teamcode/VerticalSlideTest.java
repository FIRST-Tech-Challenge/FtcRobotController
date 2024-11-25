package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class VerticalSlideTest extends LinearOpMode {
    VerticalSlides verticalSlides;

//-2724

    //total tick dist: 2907

    @Override//183
    public void runOpMode() throws InterruptedException {

        verticalSlides = new VerticalSlides(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            //verticalSlides.setSlidePower(-gamepad1.left_stick_y);
            if (gamepad1.a) {
                verticalSlides.setTargetInches(5);
                verticalSlides.goToTargetAsync();
            } else if (gamepad1.b) {
                verticalSlides.setTargetInches(10);
                verticalSlides.goToTargetAsync();
            }  else if (gamepad1.x) {
                verticalSlides.setTargetInches(0);
                verticalSlides.goToTargetAsync();
            } else {
                //verticalSlides.setTargetInches(0);
                verticalSlides.setSlidePower(-gamepad1.left_stick_y);
                verticalSlides.setTargetToCurrentPosition();
            }
            telemetry.addData("ticks", verticalSlides.currentTicks());
            telemetry.addData("ticksTarget", verticalSlides.targetTicks);
            telemetry.addData("inches", verticalSlides.currentInches());

            telemetry.update();
        }
    }
}
