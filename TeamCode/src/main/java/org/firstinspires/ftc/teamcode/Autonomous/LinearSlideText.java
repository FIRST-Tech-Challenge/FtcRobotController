package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DriveMethods;

@Autonomous(name = "LinearSlideTest", group = "LinearSlide")
public class LinearSlideText extends DriveMethods {
    public void runOpMode() {

        waitForStart();

        for(int i = 0; i < 20; i++) {
            sleep(1000);
            sleep(1000);
        }

        while(opModeIsActive()) {

        }
    }
}