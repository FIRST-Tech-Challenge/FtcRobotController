package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoCarousel;

@Autonomous
public class CarouselTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutoCarousel carousel = new AutoCarousel(hardwareMap);
        waitForStart();
        carousel.on();
        sleep(1000);
        carousel.off();
    }
}
