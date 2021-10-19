package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.chassis.Carousel;
import org.firstinspires.ftc.teamcode.chassis.MecanumChassis;

@TeleOp(name = "Mecanum OpMode w/ Carousel", group = "Remote")
public class MecanumWithCarousel extends OpMode {
    MecanumChassis chassis = new MecanumChassis();
    Carousel carousel = new Carousel();
    public void init() {
        chassis.init(hardwareMap);
        carousel.init(hardwareMap);
    }

    @Override
    public void loop() {
        chassis.drive(gamepad1);
        carousel.run(gamepad1);
    }
}
