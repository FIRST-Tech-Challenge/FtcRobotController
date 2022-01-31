package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutonomousTemplate;

/**
 * A Autonomous to test the proportional power drive capabilities of the linear slide
 */
@Disabled
@Autonomous(name = "Linear Slide Auto Test")
public class LinearSlideAutoTest extends AutonomousTemplate {
    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();

        telemetry.addData("Initialization Status", "Initialized");
        telemetry.update();

        waitForStart();
        int pos = 0;
        while (opModeIsActive() && !isStopRequested()) {

            telemetry.addData("Current Pos", slide.getEncoderCount());
            telemetry.addData("Destination", slide.getTargetHeight());
            telemetry.addData("At Pos", slide.isAtPosition(35));
            telemetry.update();

            if (slide.isAtPosition(35)) {
                pos++;
            }
            switch (pos) {
                case 0:
                    slide.setTargetLevel(HeightLevel.Down);
                    Thread.sleep(1000);
                    break;
                case 1:
                    slide.setTargetLevel(HeightLevel.BottomLevel);
                    Thread.sleep(1000);
                    break;
                case 2:
                    slide.setTargetLevel(HeightLevel.MiddleLevel);
                    Thread.sleep(1000);
                    break;

                case 3:
                    slide.setTargetLevel(HeightLevel.TopLevel);
                    Thread.sleep(1000);
                    break;
                case 4:
                    pos = 0;

            }
        }
    }
}
