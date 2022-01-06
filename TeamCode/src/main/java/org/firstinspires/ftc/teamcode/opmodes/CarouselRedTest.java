package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.mechanism.Color;

public class CarouselRedTest extends OpMode {
    Carousel carousel = new Carousel(Color.RED);
    @Override
    public void init() {
        carousel.init(hardwareMap);
    }

    @Override
    public void loop() {
        carousel.run(gamepad1);
        telemetry.addData("Current Velocity", carousel.currentVelocity);
        telemetry.addData("Target Velocity", carousel.targetVelocity);
        telemetry.addData("Velocity Error", carousel.velocityError);
    }
}
