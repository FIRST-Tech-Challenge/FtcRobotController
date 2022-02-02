package org.firstinspires.ftc.teamcode.src.drivePrograms.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.LinearSlide;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.TeleOpTemplate;

@Disabled
@TeleOp(name = "LS Test 2")
public class MoveToPosTest extends TeleOpTemplate {


    @Override
    public void opModeMain() throws InterruptedException {
        LinearSlide slide = new LinearSlide(hardwareMap, TeleOpTemplate.linearSlideMotorName);
        waitForStart();

    }
}
