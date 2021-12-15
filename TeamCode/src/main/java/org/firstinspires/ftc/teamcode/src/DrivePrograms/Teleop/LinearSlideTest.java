package org.firstinspires.ftc.teamcode.src.DrivePrograms.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.Utills.TeleopTemplate;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.LinearSlide;

@Disabled
@TeleOp(name = "LS Test")
public class LinearSlideTest extends TeleopTemplate {
    LinearSlide linearSlide;

    public void runOpMode() throws InterruptedException {
        this.initAll();
        linearSlide.setTargetLevel(LinearSlide.HeightLevels.Down);


        telemetry.addData("Initialization Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            driveTrain.setPowerFromGamepad(gamepad1);

            linearSlide.setTargetLevel(LinearSlide.HeightLevels.BottomLevel);

            //intake.setMotorPower(gamepad2.right_trigger - gamepad2.left_trigger);
            telemetry.addData("LS Height: ", linearSlide.getEncoderCount());
            telemetry.update();

            Thread.sleep(5000);
            linearSlide.setTargetLevel(LinearSlide.HeightLevels.TopLevel);

            Thread.sleep(5000);
            linearSlide.setTargetLevel(LinearSlide.HeightLevels.Down);

        }
    }
}