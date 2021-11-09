package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.mechanism.chassis.MecanumChassis;

@Autonomous(name = "Drop Box (Red)", group = "Sensor")
public class DropBoxRed extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumChassis chassis = new MecanumChassis();
    private Carousel carousel = new Carousel();

    public void runOpMode() {
        chassis.init(hardwareMap, true);
        carousel.init(hardwareMap, true);

        waitForStart();
        while (opModeIsActive()) {
            // Start button is pressed

            // Drive to the the shipping hub
            chassis.strafeLeftWithEncoders(0.6,1300);

            // Drive away, (hopefully) depositing the preloaded box in the process
            chassis.strafeRightWithEncoders(0.8,800);

            // Move to the carousel and spin it
            chassis.moveBackwardWithEncoders(0.6, 2300);
            chassis.strafeRightWithEncoders(0.6,400);
            carousel.turnCarousel();
            delay(2000);
            carousel.carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            carousel.carouselMotor.setPower(0.55);
            delay(200);
            carousel.carouselMotor.setPower(-0.55);
            delay(300);
            carousel.carouselMotor.setPower(0);

            // Drive into the warehouse
            chassis.strafeLeftWithEncoders(0.6,50);
            chassis.moveForwardWithEncoders(1, 3000);
            chassis.turnRightWithEncoders(0.6,25);
            chassis.moveForwardWithEncoders(1, 2500);


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
