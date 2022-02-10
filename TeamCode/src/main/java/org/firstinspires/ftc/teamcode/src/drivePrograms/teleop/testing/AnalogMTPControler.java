package org.firstinspires.ftc.teamcode.src.drivePrograms.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.TeleOpTemplate;

@Disabled
@TeleOp(name = "AnalogMTPControler")
public class AnalogMTPControler extends TeleOpTemplate {

    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        slide.autoMode();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                int pos = (int) (Math.abs(slide.getEncoderCount()) + (100 * -gamepad2.left_stick_y));
                telemetry.addData("Pos", pos);
                telemetry.update();
                if (pos < 0) pos = 0;
                if (pos > HeightLevel.getEncoderCountFromEnum(HeightLevel.TopLevel))
                    pos = HeightLevel.getEncoderCountFromEnum(HeightLevel.TopLevel);
                slide.setTargetPosition(pos);
            }
        }
    }
}
