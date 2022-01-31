package org.firstinspires.ftc.teamcode.src.drivePrograms.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.TeleOpTemplate;

/**
 * A Teleop to test the Linear Slide
 */

@Disabled
@TeleOp(name = "LS Test")
public class LinearSlideTest extends TeleOpTemplate {

    public void opModeMain() throws InterruptedException {
        initDriveTrain();
        initLinearSlide();

        telemetry.addData("Initialization Status", "Initialized");
        telemetry.update();

        waitForStart();
        slide.resetEncoder();
        while (opModeIsActive() && !isStopRequested()) {

            driveTrain.setPowerFromGamepad(gamepad1);

            slide.setTargetPosition((int) (slide.getTargetHeight() + (10 * -gamepad2.left_stick_y)));
            if (slide.getTargetHeight() < 0) {
                slide.setTargetPosition(0);
            }
            if (slide.getTargetHeight() > 720) {
                slide.setTargetPosition(720);
            }

            //telemetry.addData("Power", );
            telemetry.addData("LS Height: ", slide.getEncoderCount());
            telemetry.addData("Target Height: ", slide.getTargetHeight());
            telemetry.addData("Slide", slide);
            telemetry.update();

        }
    }
}