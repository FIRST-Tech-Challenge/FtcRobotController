package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.mechanism.chassis.MecanumChassis;

@Autonomous(name = "Spin Carousel (Blue)", group = "Sensor")
public class SpinCarouselBlue extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumChassis chassis = new MecanumChassis();
    private Carousel carousel = new Carousel();

    public void runOpMode() {
        chassis.init(hardwareMap, false);
        carousel.init(hardwareMap, false);

        waitForStart();
        while (opModeIsActive()) {
            // Start button is pressed

            // Move to the carousel
            chassis.moveForwardWithEncoders(0.6,700);
            chassis.strafeRightWithEncoders(0.6, 2000);
            chassis.moveBackwardWithEncoders(0.6,300);

            // Spin the carousel
            carousel.turnCarousel();
            delay(2000);
            carousel.carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            carousel.carouselMotor.setPower(0.55);
            delay(200);
            carousel.carouselMotor.setPower(-0.55);
            delay(300);
            carousel.carouselMotor.setPower(0);

            // Move into the warehouse
            chassis.moveForwardWithEncoders(0.6,50);
            chassis.strafeLeftWithEncoders(0.6,200);
            chassis.turnLeftWithEncoders(0.6,1000);
            chassis.moveForwardWithEncoders(1,5000);


            // End of auto
            break;
        }
    }

    public void delay(int time) {
        double startTime = runtime.milliseconds();
        while (runtime.milliseconds() - startTime < time) {
        }
    }
}
