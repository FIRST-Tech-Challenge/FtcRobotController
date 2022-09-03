package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Carousel;
@Disabled

@Autonomous(name="Carousel Spinner Test")

public class CarouselTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Carousel carousel = new Carousel();
        carousel.spinCarouselAutonomousBlue();
        carousel.spinCarouselAutonomousRed();

    }


}
