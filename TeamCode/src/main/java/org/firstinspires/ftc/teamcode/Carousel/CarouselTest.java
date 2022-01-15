package org.firstinspires.ftc.teamcode.Carousel;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Jon DC")
public class CarouselTest extends LinearOpMode{
    DcMotor carouselTurningMotor;

    public void runOpMode(){
        carouselTurningMotor = hardwareMap.dcMotor.get("carouselTurningMotor");


        waitForStart();

        while (opModeIsActive()) {

            Carousel car = new Carousel(hardwareMap);

            car.carouselBoolean(this.gamepad1.a);

        }
    }

}



