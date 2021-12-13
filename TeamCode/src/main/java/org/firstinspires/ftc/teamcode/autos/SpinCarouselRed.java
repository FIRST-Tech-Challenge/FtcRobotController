package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.mechanism.Color;
import org.firstinspires.ftc.teamcode.mechanism.chassis.MecanumChassis;

@Disabled
@Autonomous(name = "Spin Carousel (Red)", group = "Sensor")
public class SpinCarouselRed extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumChassis chassis = new MecanumChassis();
    private Carousel carousel = new Carousel(Color.RED);

    public void runOpMode() {
        chassis.init(hardwareMap);
        carousel.init(hardwareMap);

        waitForStart();
        // Start button is pressed

        /* old code
        // Move to the carousel
        chassis.strafeLeftWithEncoders(0.6,700);
        chassis.moveBackwardWithEncoders(0.6, 2000);
        chassis.strafeRightWithEncoders(0.6,300);

        // Spin the carousel
        carousel.turnCarousel();
        delay(2500);
        carousel.carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.carouselMotor.setPower(0);

        // Move into the warehouse
        chassis.strafeLeftWithEncoders(0.6,50);
        chassis.moveForwardWithEncoders(1,5500);*/

        // Move to the carousel
        chassis.moveForwardWithEncoders(0.6,100);
        chassis.strafeLeftWithEncoders(0.6, 950);

        // Spin the carousel
        carousel.turnCarousel();
        delay(2500);
        carousel.carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.carouselMotor.setPower(0);

        // Move into the warehouse
        chassis.strafeRightWithEncoders(0.6,50);
        chassis.moveForwardWithEncoders(0.6,250);
        chassis.turnRightWithEncoders(0.6,550);
        chassis.moveForwardWithEncoders(1,4800);
    }

    public void delay(int time) {
        double startTime = runtime.milliseconds();
        while (runtime.milliseconds() - startTime < time) {
        }
    }
}
