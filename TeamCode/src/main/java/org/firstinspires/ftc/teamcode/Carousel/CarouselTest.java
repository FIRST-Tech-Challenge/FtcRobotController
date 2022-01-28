package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Carousel Test")
public class CarouselTest extends LinearOpMode{
    DcMotor carouselTurningMotor;

    public void runOpMode(){
        carouselTurningMotor = hardwareMap.dcMotor.get("carouselTurningMotor");


        waitForStart();
        Carousel car = new Carousel(hardwareMap);

        while (opModeIsActive()) {
            car.toggleDirection(this.gamepad1.b);
            car.toggleCarousel(this.gamepad1.a);
        }


        //Hold
        // car.carouselBoolean(this.gamepad1.a, true);


        // car.carouselBoolean(this.gamepad1.b, false);

    }
}








