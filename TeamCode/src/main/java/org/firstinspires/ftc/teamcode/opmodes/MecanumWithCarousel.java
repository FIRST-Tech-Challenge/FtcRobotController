package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.mechanism.Color;
import org.firstinspires.ftc.teamcode.mechanism.chassis.MecanumChassis;

@TeleOp(name = "Mecanum OpMode w/ Carousel", group = "Remote")
public class MecanumWithCarousel extends OpMode {
    MecanumChassis chassis = new MecanumChassis();
    Carousel carousel = new Carousel(Color.RED);
    public void init() {
        // Initialize each mechanism
        chassis.init(hardwareMap);
        carousel.init(hardwareMap);
    }

    @Override
    public void loop() {
        // Run each mechanism
        chassis.run(gamepad1);
        carousel.run(gamepad1);
    }
}
