package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.chassis.MecanumChassis;

@TeleOp(name = "Mecanum OpMode w/ Carousel", group = "Remote")
public class MecanumWithCarousel extends BasicOpMode{
    public static DcMotor carousel;
    @Override
    Chassis getChassis() {
        return new MecanumChassis();
    }

    @Override
    public void init() {
        super.init();
        carousel = hardwareMap.get(DcMotor.class, "carousel");
    }

    @Override
    public void loop() {
        super.loop();

        if(gamepad1.a) {
            carousel.setPower(-0.55);
        } else {
            carousel.setPower(0);
        }
    }
}
