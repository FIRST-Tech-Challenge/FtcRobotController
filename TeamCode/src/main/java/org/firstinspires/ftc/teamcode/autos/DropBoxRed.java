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
@Autonomous(name = "Drop Box (Red)", group = "Sensor")
public class DropBoxRed extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumChassis chassis = new MecanumChassis();
    private Carousel carousel = new Carousel(Color.RED);

    public void runOpMode() {
        chassis.init(hardwareMap);
        carousel.init(hardwareMap);

        waitForStart();
        // Start button is pressed

        // Drive to the the shipping hub
        chassis.strafeLeftWithEncoders(0.6,1350);

        // Drive away, (hopefully) depositing the preloaded box in the process
        chassis.strafeRightWithEncoders(0.8,400);

        // Move to the carousel and spin it
        chassis.strafeRightWithEncoders(0.6,850);
        chassis.moveBackwardWithEncoders(0.6, 200);
        chassis.turnLeftWithEncoders(0.6,50);
        chassis.moveBackwardWithEncoders(0.6, 800);
        chassis.turnLeftWithEncoders(0.6,750);
        chassis.strafeLeftWithEncoders(0.2,200);
        carousel.turnCarousel();
        delay(2500);
        carousel.carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.carouselMotor.setPower(0);

        // Drive into the warehouse
        chassis.strafeRightWithEncoders(0.6,50);
        chassis.moveForwardWithEncoders(0.6,450);
        chassis.turnRightWithEncoders(0.6,650);
        chassis.moveForwardWithEncoders(1,4800);
    }

    public void delay(int time) {
        double startTime = runtime.milliseconds();
        while (runtime.milliseconds() - startTime < time) {
        }
    }
}
