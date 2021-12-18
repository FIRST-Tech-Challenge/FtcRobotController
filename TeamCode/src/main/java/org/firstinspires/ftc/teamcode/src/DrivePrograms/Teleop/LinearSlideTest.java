package org.firstinspires.ftc.teamcode.src.DrivePrograms.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.Utills.TeleopTemplate;


@TeleOp(name = "LS Test")
@Disabled
public class LinearSlideTest extends TeleopTemplate {

    public void runOpMode() throws InterruptedException {
        this.initAll();


        telemetry.addData("Initialization Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            driveTrain.setPowerFromGamepad(gamepad1);

            //linearSlide.setTargetLevel(LinearSlide.HeightLevel.BottomLevel);

            //intake.setMotorPower(gamepad2.right_trigger - gamepad2.left_trigger);
            if ((gamepad2.left_stick_y) != 0) {
                slide.setMotorPower(gamepad2.left_stick_y);
                slide.setTargetHeight(slide.getEncoderCount());
            } else {
                slide.threadMain();
            }
            telemetry.addData("LS Height: ", slide.getEncoderCount());
            telemetry.addData("Target Height: ", slide.getTargetHeight());
            telemetry.update();

            //Thread.sleep(5000);
            //linearSlide.setTargetLevel(LinearSlide.HeightLevel.TopLevel);

            //Thread.sleep(5000);
            //linearSlide.setTargetLevel(LinearSlide.HeightLevel.Down);

        }
    }
}