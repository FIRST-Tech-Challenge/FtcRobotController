package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.mechanism.Color;
import org.firstinspires.ftc.teamcode.mechanism.chassis.MecanumChassis;

@Autonomous(name = "Drop Box (Blue)", group = "Sensor")
public class DropBoxBlue extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumChassis chassis = new MecanumChassis();
    private Carousel carousel = new Carousel(Color.BLUE);

    public void runOpMode() {
        chassis.init(hardwareMap);
        carousel.init(hardwareMap);

        waitForStart();
        // Start button is pressed

        /* old code
        // Drive to the the shipping hub
        chassis.strafeRightWithEncoders(0.6,1350);

        // Drive away, (hopefully) depositing the preloaded box in the process
        chassis.strafeLeftWithEncoders(0.8,400);

        // Move to the carousel and spin it
        chassis.strafeLeftWithEncoders(0.6,850);
        chassis.moveBackwardWithEncoders(0.6, 200);
        chassis.turnRightWithEncoders(0.6,50);
        chassis.moveBackwardWithEncoders(0.6, 800);
        chassis.turnRightWithEncoders(0.6,750);
        chassis.strafeRightWithEncoders(0.2,200);
        carousel.turnCarousel();
        delay(2500);
        carousel.carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.carouselMotor.setPower(0);

        // Drive into the warehouse
        chassis.strafeLeftWithEncoders(0.6,50);
        chassis.moveForwardWithEncoders(0.6,450);
        chassis.turnLeftWithEncoders(0.6,650);
        chassis.moveForwardWithEncoders(1,4800);
         */

        // Drive to the the shipping hub
        chassis.strafeRightWithEncoders(0.6,1300);

        // Drive away, (hopefully) depositing the preloaded box in the process
        chassis.strafeLeftWithEncoders(0.8,800);

        // Move to the carousel and spin it
        chassis.moveBackwardWithEncoders(0.6, 2300);
        chassis.strafeLeftWithEncoders(0.6,400);
        carousel.turnCarousel();
        delay(2500);
        carousel.carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.carouselMotor.setPower(0);

        // Drive into the warehouse
        chassis.strafeRightWithEncoders(0.6,50);
        chassis.moveForwardWithEncoders(1, 3000);
        chassis.turnLeftWithEncoders(0.6,25);
        chassis.moveForwardWithEncoders(1, 2500);
    }

    public void delay(int time) {
        double startTime = runtime.milliseconds();
        while (runtime.milliseconds() - startTime < time) {
        }
    }
}
